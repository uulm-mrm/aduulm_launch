from __future__ import annotations
from typing import Dict, Any, Tuple, List, Callable, Generator, Optional, Concatenate, ParamSpec, cast
from dataclasses import fields, _MISSING_TYPE, is_dataclass
from functools import lru_cache
import re
import yaml
import argparse


from dataclasses import dataclass, field
from typing import Any, List
from typing import TypeVar, Generic, Callable, Dict, ParamSpec, Concatenate
from itertools import chain

Topic = str


K = TypeVar('K')
V = TypeVar('V')


class LaunchConfigException(Exception):
    pass


class SaferDict(Generic[K, V], dict[K, V]):
    # only allow __setitem__ if key already exists
    def __setitem__(self, key: K, value: V):
        if key not in self:
            raise LaunchConfigException(
                f"Could not replace argument {key} because it does not exist!")
        super().__setitem__(key, value)

    # only allow add if key does not already exist
    def add(self, key: K, value: V):
        if key in self:
            raise LaunchConfigException(
                f"Could not add argument {key} because it already exists!")
        super().__setitem__(key, value)
        return value

    def toparamdict(self):
        def fix(val):
            if isinstance(val, bool) or isinstance(val, float) or isinstance(val, int):
                return val
            if isinstance(val, list) or isinstance(val, tuple):
                return [fix(v) for v in val]
            return str(val)
        return {k: fix(v) for k, v in self.items()}

    def tostrdict(self):
        return {k: str(v) for k, v in self.items()}


@dataclass(slots=True)
class Executable:
    executable: str
    args: List[str] = field(default_factory=list)
    output: str = 'screen'
    emulate_tty: bool = True
    xterm: bool = False
    gdb: bool = False
    respawn: bool = False
    respawn_delay: float = 0.0


@dataclass(slots=True)
class PublisherInfo:
    topic: str


@dataclass(slots=True)
class ServiceClientInfo:
    topic: str


@dataclass(slots=True)
class SubscriberInfo:
    topic: str
    outputs: List[PublisherInfo] = field(default_factory=list)
    service_calls: List[ServiceClientInfo] = field(default_factory=list)
    changes_dataprovider_state: bool = False


@dataclass(slots=True)
class ServiceInfo:
    topic: str
    outputs: List[PublisherInfo] = field(default_factory=list)
    service_calls: List[ServiceClientInfo] = field(default_factory=list)
    changes_dataprovider_state: bool = False


@dataclass(slots=True)
class TimerInfo:
    period: float
    outputs: List[PublisherInfo] = field(default_factory=list)
    service_calls: List[ServiceClientInfo] = field(default_factory=list)
    changes_dataprovider_state: bool = False


@dataclass(slots=True)
class Node:
    package: str
    executable: str
    parameters: SaferDict[str, Any] = field(default_factory=SaferDict)
    args: List[str] = field(default_factory=list)
    output: str = 'screen'
    emulate_tty: bool = True
    xterm: bool = False
    gdb: bool = False
    respawn: bool = False
    respawn_delay: float = 0.0

    publishers: Dict[str, PublisherInfo] = field(default_factory=dict)
    subscribers: Dict[str, SubscriberInfo] = field(default_factory=dict)
    service_clients: Dict[str, ServiceClientInfo] = field(default_factory=dict)
    services: Dict[str, ServiceInfo] = field(default_factory=dict)
    timers: List[TimerInfo] = field(default_factory=list)

    def get_remappings(self):
        remappings: dict[str, str] = {}
        for name, node in chain(self.publishers.items(), self.subscribers.items(), self.service_clients.items(), self.services.items()):
            remappings[name] = node.topic
        return remappings


@dataclass(slots=True)
class SubLaunchROS:
    package: str
    launchfile: str
    args: SaferDict[str, Any] = field(default_factory=SaferDict)


@dataclass
class LaunchGroup:
    modules: SaferDict[str, 'Executable | Node | SubLaunchROS | LaunchGroup'] = field(
        default_factory=SaferDict)


LeafLaunch = Executable | Node | SubLaunchROS
AnyLaunch = LeafLaunch | LaunchGroup


class LaunchConfig:
    def __init__(self, config: LaunchConfig | None = None, path: Optional[List[str]] = None, data: Optional[LaunchGroup] = None, val: Optional[LeafLaunch] = None):
        self._config = config if config is not None else self
        self._path = path if path is not None else []
        self._data = data if data is not None else LaunchGroup()
        self._val: Optional[LeafLaunch] = val
        # key -> [Value, Usage count]
        self._overrides: Dict[str, Tuple[Any, int]] = {}
        self._param_overrides: Dict[str, Tuple[Any, int]] = {}

    def _getconfig(self) -> LaunchConfig:
        return object.__getattribute__(self, '_config')

    def _getpath(self) -> List[str]:
        return object.__getattribute__(self, '_path')

    def _getdata(self) -> LaunchGroup:
        return object.__getattribute__(self, '_data')

    def _getval(self) -> Optional[LeafLaunch]:
        return object.__getattribute__(self, '_val')

    def _getold_data(self) -> LaunchGroup:
        return object.__getattribute__(self, '_old_data')

    def _getold_path(self) -> List[str]:
        return object.__getattribute__(self, '_old_path')

    def _getoverrides(self) -> Dict[str, Tuple[Any, int]]:
        return object.__getattribute__(self, '_overrides')

    def _getparam_overrides(self) -> Dict[str, Tuple[Any, int]]:
        return object.__getattribute__(self, '_param_overrides')

    def __enter__(self):
        assert self._getval() is None
        self._old_data = self._getconfig()._getdata()
        self._old_path = self._getconfig()._getpath()
        self._getconfig()._data = self._data
        self._getconfig()._path = self._getpath()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._getconfig()._data = self._old_data
        self._getconfig()._path = self._getold_path()

    def resolve_topic(self, topic: str):
        assert self._getval() is None
        # do not add namespaces to absolute topics
        if topic.startswith('/'):
            return topic
        # add namespaces for relative topics
        return '/' + '/'.join(self._getpath() + [topic])

    def resolve_namespace(self):
        assert self._getval() is None
        return '/' + '/'.join(self._getpath())

    def group(self, group_name: str):
        assert self._getval() is None
        # check if group exists, otherwise add group
        data = self._getdata()
        if group_name in data.modules:
            group = data.modules[group_name]
            if not isinstance(group, LaunchGroup):
                raise LaunchConfigException(
                    f'trying to enter group {group_name}, but it already exists and is not a group! Type: {type(group)}')
        else:
            group = LaunchGroup()
            data.modules.add(group_name, group)
        return LaunchConfig(self._getconfig(), self._getpath() + [group_name], group)

    def add(self, name: str, child: LeafLaunch):
        assert self._getval() is None
        if name in self._getdata().modules:
            raise LaunchConfigException(
                f'trying to add {name} to current group, but something already exists there! {self._data.modules[name]}')
        self._getdata().modules.add(name, child)

    def add_sublaunch_ros(self, name: str, package: str, launchfile: str, args: Dict[str, Any] = {}):
        assert self._getval() is None
        sublaunch = SubLaunchROS(package, launchfile, args=SaferDict(**args))
        self.add(name, sublaunch)
        return sublaunch

    def add_node(self, name: str, package: str, executable: str,
                 parameters: Dict[str, Any] = {}, args: List[str] = [], output: str = 'screen', emulate_tty: bool = True, respawn: bool = False, respawn_delay: float = 0.0):
        assert self._getval() is None
        params = SaferDict(**parameters)
        self._insert_param_overrides(name, params)
        node = Node(package, executable, parameters=params,
                    args=args[:], output=output, emulate_tty=emulate_tty, respawn=respawn, respawn_delay=respawn_delay)
        self.add(name, node)
        return node

    def add_executable(self, name: str, executable: str, args: List[str] = [], output: str = 'screen', emulate_tty: bool
                       = True, respawn: bool = False, respawn_delay: float = 0.0):
        assert self._getval() is None
        node = Executable(executable, args=args[:], output=output,
                          emulate_tty=emulate_tty, respawn=respawn, respawn_delay=respawn_delay)
        self.add(name, node)
        return node

    def recurse(self, cb_leaf: RecurseLeafFunc, cb_enter: Optional[RecurseEnterFunc] = None, cb_exit:
                Optional[RecurseExitFunc] = None):
        assert self._getval() is None
        self._do_recurse(self, self._getdata(), None, None,
                         cb_leaf, cb_enter, cb_exit)

    def _do_recurse(self, config: LaunchConfig, mod: AnyLaunch, name: Optional[str], parent: Optional[LaunchGroup],
                    cb_leaf: RecurseLeafFunc, cb_enter: Optional[RecurseEnterFunc] = None, cb_exit: Optional[RecurseExitFunc] = None):
        assert self._getval() is None
        if isinstance(mod, LaunchGroup):
            old_config = config
            if name is not None:
                config = config.group(name)
                if cb_enter is not None:
                    cb_enter(config, mod, name, parent)
            old_keys = set(mod.modules.keys())
            for key in old_keys:
                self._do_recurse(
                    config, mod.modules[key], key, mod, cb_leaf, cb_enter)
            # in case new modules were added
            new_keys = set(mod.modules.keys())
            for key in new_keys-old_keys:
                self._do_recurse(
                    config, mod.modules[key], key, mod, cb_leaf, cb_enter)
            if name is not None and cb_exit is not None:
                cb_exit(old_config, mod, name, parent)
        else:
            assert isinstance(name, str)
            assert parent is not None
            cb_leaf(config, mod, name, parent)

    def items(self) -> Generator[Tuple[str, LaunchConfig | LeafLaunch], None, None]:
        assert self._getval() is None
        for key, mod in self._getdata().modules.items():
            if isinstance(mod, LaunchGroup):
                yield key, self.group(key)
            else:
                yield key, mod

    def iter_leaves(self) -> Generator[Tuple[str, LeafLaunch], None, None]:
        assert self._getval() is None
        for key, mod in self._getdata().modules.items():
            if not isinstance(mod, LaunchGroup):
                yield key, mod

    def iter_groups(self) -> Generator[Tuple[str, LaunchConfig], None, None]:
        assert self._getval() is None
        for key, mod in self._getdata().modules.items():
            if isinstance(mod, LaunchGroup):
                yield key, self.group(key)

    def __iter__(self) -> Generator[str, None, None]:
        assert self._getval() is None
        yield from self._getdata().modules.__iter__()

    def values(self) -> Generator[LaunchConfig | LeafLaunch, None, None]:
        assert self._getval() is None
        for _, mod in self.items():
            yield mod

    def __repr__(self):
        assert self._getval() is None
        return repr(self._getdata())

    def __str__(self):
        assert self._getval() is None
        return str(self._getdata())

    def __getattr__(self, key):
        assert self._getval() is None
        if key not in self._getdata().modules:
            raise AttributeError(key)
        val = self._getdata().modules[key]
        if isinstance(val, LaunchGroup):
            return self.group(key)
        return LaunchConfig(self, self._getpath(), self._getdata(), val)

    def get_leaf(self) -> LeafLaunch:
        val = self._getval()
        assert val is not None
        return val

    def get_node(self):
        val = self._getval()
        assert val is not None
        assert isinstance(val, Node)
        return val

    def get_group(self):
        assert self._getval() is None
        return self

    def get_executable(self):
        val = self._getval()
        assert val is not None
        assert isinstance(val, Executable)
        return val

    def get_sublaunch_ros(self):
        val = self._getval()
        assert val is not None
        assert isinstance(val, SubLaunchROS)
        return val

    def overrides(self):
        return OverrideSetter(self._getoverrides(), self._getpath())

    def param_overrides(self):
        return OverrideSetter(self._getparam_overrides(), self._getpath())

    def _insert_param_overrides(self, node_name: str, args: SaferDict[str, Any]):
        overrides = self._getparam_overrides()

        def split_pattern(pattern: str):
            res = pattern.split('.')
            return '.'.join(res[:-1]), res[-1]

        split_overrides = [(k, *split_pattern(k), v)
                           for k, v in overrides.items()]
        key = '.'.join(self._getpath() + [node_name])
        for orig_k, k, param_name, (v, usage_cnt) in split_overrides:
            if not matches(key, k):
                continue
            args.add(param_name, v)
            overrides[orig_k] = (v, usage_cnt+1)

    def insert_overrides(self, params: Any):
        assert is_dataclass(params)
        overrides = self._getoverrides()
        for field in fields(params):
            key = '.'.join(self._getpath() + [field.name])
            m = [(k, v) for k, v in overrides.items() if matches(key, k)]
            if len(m) == 0:
                continue
            # should not have been inserted if multiple patterns match
            assert len(m) == 1
            k, (v, usage_cnt) = m[0]
            if not isinstance(field.default, _MISSING_TYPE):
                default = field.default
            else:
                assert not isinstance(field.default_factory, _MISSING_TYPE)
                default = field.default_factory()
            if not isinstance(v, field.type):
                raise LaunchConfigException(
                    f'Attribute {field.name} of {params.__class__} was overridden by override {k} with value {v} but type should be {field.type}!')
            if getattr(params, field.name) != default:
                raise LaunchConfigException(
                    f'Attribute {field.name} of {params.__class__} was already assigned to and can not be overridden by override {k}!')
            setattr(params, field.name, v)
            overrides[k] = (v, usage_cnt+1)

    def check_overrides_counts(self):
        self._check_overrides_counts(self._getoverrides(), 'argument')
        self._check_overrides_counts(self._getparam_overrides(), 'parameter')

    def _check_overrides_counts(self, overrides: Dict[str, Tuple[Any, int]], _type: str):
        unused_overrides = [k for k, (_, e) in overrides.items() if e == 0]
        if len(unused_overrides) != 0:
            raise LaunchConfigException(
                f'The following {_type} overrides were specified but were not applied. Maybe a typo or someone forgot to call insert_overrides()? {unused_overrides}')

    def parse_args(self, args: List[str], params: List[str]):
        def parse(lst: List[str], overrides: Dict[str, Tuple[Any, int]]):
            for arg in lst:
                assert ':=' in arg
                key, val = arg.split(':=')
                check_override_valid(key, overrides)
                val = yaml.load(val, Loader=yaml.SafeLoader)
                overrides[key] = (val, 0)

        parse(args, self._getoverrides())
        parse(params, self._getparam_overrides())

    def parse_argv(self):
        parser = argparse.ArgumentParser()
        parser.add_argument('args', type=str, nargs='*', default=[])
        parser.add_argument('--params', type=str, nargs='+', default=[])
        args = parser.parse_args()
        self.parse_args(args.args, args.params)

    P1 = ParamSpec('P1')

    def add_publisher(self, node: Node, name: str, _constructor: Callable[Concatenate[P1], PublisherInfo] = PublisherInfo, *args: P1.args, **kwargs: P1.kwargs):
        info = _constructor(*args, **kwargs)
        info.topic = self.resolve_topic(info.topic)
        node.publishers[name] = info
        return info

    def add_subscriber(self, node: Node, name: str, _constructor: Callable[Concatenate[P1], SubscriberInfo] = SubscriberInfo, *args: P1.args, **kwargs: P1.kwargs):
        info = _constructor(*args, **kwargs)
        info.topic = self.resolve_topic(info.topic)
        node.subscribers[name] = info
        return info

    def add_service_client(self, node: Node, name: str, _constructor: Callable[Concatenate[P1], ServiceClientInfo] = ServiceClientInfo, *args: P1.args, **kwargs: P1.kwargs):
        info = _constructor(*args, **kwargs)
        info.topic = self.resolve_topic(info.topic)
        node.service_clients[name] = info
        return info

    def add_service(self, node: Node, name: str, _constructor: Callable[Concatenate[P1], ServiceInfo] = ServiceInfo, *args: P1.args, **kwargs: P1.kwargs):
        info = _constructor(*args, **kwargs)
        info.topic = self.resolve_topic(info.topic)
        node.services[name] = info
        return info

    def add_timer(self, node: Node, _constructor: Callable[Concatenate[P1], TimerInfo] = TimerInfo, *args: P1.args, **kwargs: P1.kwargs):
        info = _constructor(*args, **kwargs)
        node.timers.append(info)
        return info


@lru_cache(maxsize=256, typed=True)
def _compile_fnmatch(pat):
    pat2 = pat.replace('__', '.*').replace('_/', '[^/]*/')
    pat2 = pat2 + '$'
    return re.compile(pat2).match


def fnmatch(name, pat):
    return _compile_fnmatch(str(pat))(str(name)) is not None


def matches(key: str, pattern: str):
    key = key.replace('.', '/')
    pattern = pattern.replace('.', '/')
    return fnmatch(key, pattern)


def check_override_valid(key: str, overrides: Dict[str, Tuple[Any, int]]):
    if key in overrides:
        raise LaunchConfigException(f'Override for "{key}" already exists!')
    if len([pattern for pattern, _ in overrides.items() if matches(key, pattern)]) > 0:
        raise LaunchConfigException(
            f'Override for "{key}" is already matched by previous glob patterns!')
    if len([pattern for pattern, _ in overrides.items() if matches(pattern, key)]) > 0:
        raise LaunchConfigException(
            f'Glob pattern override for "{key}" matches existing patterns!')


class OverrideSetter:
    def __init__(self, overrides: Dict[str, Tuple[Any, int]], path: List[str]):
        object.__setattr__(self, '_overrides', overrides)
        object.__setattr__(self, '_path', path)

    def __setattr__(self, key: str, value: Any):
        key = '.'.join(self._getpath() + [key])
        overrides = self._getoverrides()
        check_override_valid(key, overrides)
        self._getoverrides()[key] = (value, 0)

    def __getattr__(self, key: str):
        return OverrideSetter(self._getoverrides(), self._getpath() + [key])

    def _getpath(self) -> List[str]:
        return object.__getattribute__(self, '_path')

    def _getoverrides(self) -> Dict[str, Tuple[Any, int]]:
        return object.__getattribute__(self, '_overrides')


RecurseLeafFunc = Callable[[LaunchConfig, LeafLaunch, str, LaunchGroup], None]
RecurseEnterFunc = Callable[[LaunchConfig,
                             LaunchGroup, str, Optional[LaunchGroup]], None]
RecurseExitFunc = Callable[[LaunchConfig,
                            LaunchGroup, str, Optional[LaunchGroup]], None]
