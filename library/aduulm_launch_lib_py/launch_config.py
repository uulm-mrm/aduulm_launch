from __future__ import annotations

import io
import re
from dataclasses import fields, _MISSING_TYPE, is_dataclass, field, dataclass, \
    Field, MISSING
from enum import IntEnum
from functools import lru_cache
from itertools import chain
from pathlib import Path, PurePath
from types import UnionType
from typing import cast, Dict, Any, Tuple, List, Callable, Generator, Concatenate, \
    ParamSpec, Literal, Optional, TypeVar, Generic, Union, get_args, get_origin

import yaml

K = TypeVar('K')
V = TypeVar('V')


class LaunchConfigException(Exception):
    pass


SEPARATOR_NAMESPACE = '.'
SEPARATOR_ATTRIBUTE = '.'
_isgenericpattern = re.compile(r'(^|\.)__?\.')


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
        def fix(key, val):
            if isinstance(val, LogLevel):
                return int(val)
            if isinstance(val, bool) or isinstance(val, float) or isinstance(val, int):
                return val
            if isinstance(val, dict):
                return {fix(None, k): fix(fix(None, k), v)
                        for k, v in val.items()}
            if isinstance(val, list) or isinstance(val, tuple):
                res = [fix(None, v) for v in val]
                if len(res) == 0:
                    print("Self:", self)
                    print("Offending entry:", key, val)
                    raise LaunchConfigException(
                        'Received an empty list/tuple as a parameter value. ROS will probably throw an error here. Maybe pass a non-empty list with a dummy element (empty string) and handle this special case in the Node.')
                return res
            return str(val)
        return {k: fix(k, v) for k, v in self.items()}

    def tostrdict(self):
        return {k: str(v) for k, v in self.items()}


class LogLevel(IntEnum):
    Fatal = 0
    Error = 1
    Warning = 2
    Info = 3
    Debug = 4


@dataclass(slots=True)
class Executable:
    executable: str
    args: List[str] = field(default_factory=list)
    output: str = 'screen'
    emulate_tty: bool = True
    xterm: bool = False
    gdb: bool = False
    valgrind: bool = False
    respawn: bool = False
    respawn_delay: float = 0.0
    required: bool = False
    set_name: bool = True


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
class TimeSyncInfo:
    names: List[str]
    topics: List[str]
    queue_size: int
    slop: Optional[float] = None
    outputs: List[PublisherInfo] = field(default_factory=list)
    service_calls: List[ServiceClientInfo] = field(default_factory=list)
    changes_dataprovider_state: bool = False

    def __post_init__(self):
        assert len(self.names) == len(self.topics)
        assert len(self.names) >= 2


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
class ROSLoggerInfo:
    node_name: str
    log_level: LogLevel


Logger = ROSLoggerInfo
OutputType = Literal['screen'] | Literal['both'] | Literal['log']


@dataclass(slots=True)
class TritonModel:
    model_path: Path
    # relative paths will be interpreted relative to model_path
    config_path: PurePath = PurePath('config.pbtxt')


@dataclass(slots=True)
class Node:
    package: str
    executable: str
    parameters: SaferDict[str, Any] = field(default_factory=SaferDict)
    args: List[str] = field(default_factory=list)
    output: OutputType = 'screen'
    emulate_tty: bool = True
    xterm: bool = False
    gdb: bool = False
    valgrind: bool = False
    respawn: bool = False
    respawn_delay: float = 0.0
    required: bool = False
    handle_lifecycle: bool | List[Union[Literal['activation'],
                                        Literal['configuration']]] = False
    metadata: SaferDict[str, Any] = field(default_factory=SaferDict)
    additional_env: SaferDict[str, Any] = field(default_factory=SaferDict)
    triton_models: List[TritonModel] = field(default_factory=list)
    set_name: bool = True

    publishers: Dict[str, PublisherInfo] = field(default_factory=dict)
    subscribers: Dict[str, SubscriberInfo] = field(default_factory=dict)
    time_sync_subscribers: List[TimeSyncInfo] = field(default_factory=list)
    service_clients: Dict[str, ServiceClientInfo] = field(default_factory=dict)
    services: Dict[str, ServiceInfo] = field(default_factory=dict)
    timers: List[TimerInfo] = field(default_factory=list)
    loggers: List[Logger] = field(default_factory=list)

    def get_remappings(self):
        remappings: dict[str, str] = {}
        for name, node in chain(self.publishers.items(), self.subscribers.items(), self.service_clients.items(), self.services.items()):
            remappings[name] = node.topic
        for node in self.time_sync_subscribers:
            for name, topic in zip(node.names, node.topics):
                remappings[name] = topic
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
    ros2_entities: List[LaunchDescriptionEntity] = field(default_factory=list)


LeafLaunch = Executable | Node | SubLaunchROS
AnyLaunch = LeafLaunch | LaunchGroup

OverridesT = Dict[str, Tuple[Any, int, List[Any]]]


@dataclass(slots=True, kw_only=True)
class OverridableField:
    name: str
    field_type: type
    default_value: Any
    value: Any


class LaunchConfig:
    def __init__(self, config: LaunchConfig | None = None, path: Optional[List[str]] = None, data: Optional[LaunchGroup] = None, val: Optional[LeafLaunch] = None):
        self._config = config if config is not None else self
        self._path = path if path is not None else []
        self._data = data if data is not None else LaunchGroup()
        self._val: Optional[LeafLaunch] = val
        # key -> [Value, Usage count]
        self._overrides: OverridesT = {}
        self._param_overrides: OverridesT = {}
        self._avail_overrides: List[Tuple[type, List[OverridableField]]] = []

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

    def _getoverrides(self) -> OverridesT:
        return object.__getattribute__(self, '_overrides')

    def _getparam_overrides(self) -> OverridesT:
        return object.__getattribute__(self, '_param_overrides')

    def _getavail_overrides(self) -> List[Tuple[type, List[OverridableField]]]:
        return object.__getattribute__(self, '_avail_overrides')

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
        return f'{self.resolve_namespace()}/{topic}'

    def resolve_namespace(self):
        assert self._getval() is None
        path = self._getpath()
        if len(path) == 0:
            return ''
        return '/' + '/'.join(path)

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
                 parameters: Dict[str, Any] = {}, args: List[str] = [], output: OutputType = 'screen', emulate_tty: bool
                 = True, respawn: bool = False, respawn_delay: float = 0.0, required: bool = False, handle_lifecycle:
                 bool | List[Union[Literal['activation'], Literal['configuration']]] = False, set_name: bool = True,
                 metadata: dict[str, Any] = {}, additional_env: dict[str, Any] = {}):
        assert self._getval() is None
        params = SaferDict(**parameters)
        metadata = SaferDict(**metadata)
        additional_env = SaferDict(**additional_env)
        self._insert_param_overrides(name, params)
        node = Node(package, executable, parameters=params,
                    args=args[:], output=output, emulate_tty=emulate_tty, respawn=respawn, respawn_delay=respawn_delay,
                    required=required, handle_lifecycle=handle_lifecycle, set_name=set_name, metadata=metadata,
                    additional_env=additional_env)
        self.add(name, node)
        return node

    def add_executable(self, name: str, executable: str, args: List[str] = [], output: OutputType = 'screen', emulate_tty: bool
                       = True, respawn: bool = False, respawn_delay: float = 0.0, set_name: bool = True, required: bool = False):
        assert self._getval() is None
        node = Executable(executable, args=args[:], output=output,
                          emulate_tty=emulate_tty, respawn=respawn, respawn_delay=respawn_delay, set_name=set_name, required=required)
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
        for orig_k, k, param_name, (v, usage_cnt, lst) in split_overrides:
            if not matches(key, k):
                continue
            args.add(param_name, v)
            overrides[orig_k] = (v, usage_cnt+1, lst)

    def _get_matching_override(self, path: List[str], field: Field[Any], overrides: OverridesT):
        key = SEPARATOR_ATTRIBUTE.join([*path, field.name])
        if len(self._getpath()) > 0:
            key = SEPARATOR_NAMESPACE.join(
                self._getpath()) + SEPARATOR_ATTRIBUTE + key
        m = [(k, v) for k, v in overrides.items() if matches(key, k)]
        if len(m) == 0:
            return
        # should not have been inserted if multiple patterns match
        assert len(m) == 1
        k, (v, usage_cnt, lst) = m[0]
        return k, v, usage_cnt, lst

    # get overrides at current path for dataclass
    def get_overrides(self, params_t: type):
        assert is_dataclass(params_t)
        overrides = self._getoverrides()

        # https://stackoverflow.com/questions/74544539/python-how-to-check-what-types-are-in-defined-types-uniontype
        def is_union(t: object) -> bool:
            origin = get_origin(t)
            return origin is Union or origin is UnionType

        def accepts_type(t: type, test_t: type):
            if is_union(t):
                return sum([accepts_type(t_, test_t) for t_ in get_args(t)]) > 0
            origin = get_origin(t)
            if origin is not None:
                return issubclass(test_t, origin)
            return issubclass(test_t, t)

        res: List[Tuple[str, List[str], Field[Any], Any, List[Any]]] = []

        def check_field(params_t: type, field: Field[Any], path: List[str] = []):
            if is_dataclass(field.type):
                for f in fields(field.type):
                    check_field(field.type, f, path + [field.name])
                return
            _ov = self._get_matching_override(path, field, overrides)
            if _ov is None:
                return
            k, v, _, lst = _ov
            # Path is derived from Purepath, so check for Path first
            if isinstance(v, str) and accepts_type(field.type, Path):
                v = Path(v)
            elif isinstance(v, str) and accepts_type(field.type, PurePath):
                v = PurePath(v)
            elif isinstance(v, str) and accepts_type(field.type, LogLevel):
                log_level_name = v[0].upper() + v[1:].lower()
                try:
                    v = LogLevel[log_level_name]
                except KeyError:
                    raise LaunchConfigException(
                        f'Attribute {field.name} of {params_t} was overridden by override {k} with invalid value \'{v}\', valid values are {[v.name for v in LogLevel]}!')
            if not accepts_type(field.type, type(v)):
                raise LaunchConfigException(
                    f'Attribute {field.name} of {params_t} was overridden by override {k} with value {v} of type {type(v)} but type should be {field.type}!')
            res.append((k, [*path, field.name], field, v, lst))

        for field in fields(params_t):
            check_field(params_t, field)
        return res

    def _insert_avail_overrides(self, params: Any):
        params_t = type(params)
        assert is_dataclass(params_t)
        lst = []

        def visit_field(f: Field[Any], value: Any, path: List[str]):
            if is_dataclass(f.type):
                for f in fields(f.type):
                    visit_field(f, getattr(value, f.name), path + [f.name])
                return
            abs_path = '.'.join([*path, f.name])
            default_val = f.default if f.default != MISSING \
                else f.default_factory() if f.default_factory != MISSING \
                else None
            lst.append(OverridableField(
                name=abs_path, field_type=f.type, default_value=default_val,
                value=value))

        for f in fields(params_t):
            visit_field(f, getattr(params, f.name), self._getpath())
        self._avail_overrides.append((params_t, lst))

    ParamsT = TypeVar('ParamsT')

    def instantiate_dataclass_from_overrides(self, params_t: Callable[..., ParamsT]) -> ParamsT:
        overrides = self._getoverrides()

        def instantiate(t: Any, path: List[str]):
            assert is_dataclass(t)
            values = {}
            used = []
            for field in fields(t):
                if is_dataclass(field.type):
                    values[field.name] = instantiate(
                        field.type, path + [field.name])
                    continue
                _ov = self._get_matching_override(path, field, overrides)
                if _ov is None:
                    continue
                k, v, *_ = _ov
                values[field.name] = v
                used.append(k)
            try:
                params = t(**values)
            except TypeError as e:
                raise LaunchConfigException(
                    f'Could not construct instance of dataclass type {t}! Probably the class has required fields but no override was provided!') from e
            for k in used:
                self.inc_override_count(k, params)
            return params
        return cast(Any, instantiate(params_t, []))

    def inc_override_count(self, k: str, params: Any):
        overrides = self._getoverrides()
        old_v, usage_cnt, lst = overrides[k]
        if params not in lst:
            lst.append(params)
        overrides[k] = (old_v, usage_cnt+1, lst)

    def insert_overrides(self, params: Any):
        assert is_dataclass(params)
        for k, path, field, v, lst in self.get_overrides(type(params)):
            if not isinstance(field.default, _MISSING_TYPE):
                default = field.default
            else:
                if isinstance(field.default_factory, _MISSING_TYPE):
                    default = None
                else:
                    default = field.default_factory()
            struct = params
            for p in path[:-1]:
                struct = getattr(struct, p)
            val = getattr(struct, path[-1])
            if val != default and val != v and params not in lst:
                print(
                    f'Warning: Attribute {field.name} of {params.__class__} was already assigned the value {val} and was now overriden by override {k} with value {v}')
            setattr(struct, path[-1], v)
            self.inc_override_count(k, struct)
        self._insert_avail_overrides(params)

    def check_overrides_counts(self):
        self._check_overrides_counts(self._getoverrides(), 'argument')
        self._check_overrides_counts(self._getparam_overrides(), 'parameter')

    def _check_overrides_counts(self, overrides: OverridesT, _type: str):
        unused_overrides = [k for k, (_, e, _) in overrides.items() if e == 0]
        if len(unused_overrides) != 0:
            raise LaunchConfigException(
                f'The following {_type} overrides were specified but were not applied. Maybe a typo or someone forgot to call insert_overrides()? {unused_overrides}')
        multiply_used_overrides = [(k, lst) for k, (_, e, lst) in overrides.items(
        ) if e > 1 and _isgenericpattern.search(k) is None and (len(lst) > 1 or _type == 'parameter')]
        if len(multiply_used_overrides) != 0:
            print(f'The following {_type} overrides were applied multiple times although they do not contain wildcards. This is not an error, but make sure that your launch file is really correct.')
            for k, lst in multiply_used_overrides:
                print(f"{k} is used by the following dataclasses:")
                for e in lst:
                    print(f"  {e}")
                print()

    def load_overrides_from_yaml(self, overrides_file: Path):
        overrides = self._getoverrides()
        with open(overrides_file, 'r') as f:
            overrides_data = yaml.load(f, Loader=yaml.SafeLoader)
        if overrides_data is None:
            return

        def _yaml_iter(elem, prefix: Optional[str] = None):
            if isinstance(elem, dict):
                for key, val in elem.items():
                    _yaml_iter(
                        val, key if prefix is None else f'{prefix}.{key}')
                return
            assert prefix is not None
            check_override_valid(prefix, overrides)
            overrides[prefix] = (elem, 0, [])

        _yaml_iter(overrides_data)

    def parse_args(self, args: List[str], params: List[str], overrides_file: Optional[Path] = None):
        def parse(lst: List[str], overrides: OverridesT):
            for arg in lst:
                assert ':=' in arg
                key, val = arg.split(':=')
                check_override_valid(key, overrides)
                val = yaml.load(val, Loader=yaml.SafeLoader)
                overrides[key] = (val, 0, [])

        parse(args, self._getoverrides())
        parse(params, self._getparam_overrides())
        if overrides_file is not None:
            self.load_overrides_from_yaml(overrides_file)

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

    def add_time_sync_subscriber(self, node: Node, _constructor: Callable[Concatenate[P1], TimeSyncInfo] = TimeSyncInfo, *args: P1.args, **kwargs: P1.kwargs):
        info = _constructor(*args, **kwargs)
        info.topics = [self.resolve_topic(topic) for topic in info.topics]
        node.time_sync_subscribers.append(info)
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

    def add_ros_logger(self, node: Node, node_name: str, log_level: LogLevel):
        info = ROSLoggerInfo(node_name=node_name, log_level=log_level)
        node.loggers.append(info)
        return info

    def add_ros2_entity(self, ros2_entity: LaunchDescriptionEntity):
        assert self._getval() is None
        self._getdata().ros2_entities.append(ros2_entity)

    def generate_topic_graphviz(self, f: io.TextIO):
        @dataclass
        class TopicInfo:
            name: str
            sub_nodes: List[str] = field(default_factory=list)
            pub_nodes: List[str] = field(default_factory=list)
            srv_nodes: List[str] = field(default_factory=list)
            scl_nodes: List[str] = field(default_factory=list)

        topic_infos: Dict[str, TopicInfo] = dict()

        # generate orchestrator launch config and remappings
        def cb(cfg: LaunchConfig, mod: LeafLaunch, name: str, _: LaunchGroup):
            if not isinstance(mod, Node):
                return

            node_name = f'{cfg.resolve_namespace()}/{name}'

            for list_name, field_name in [
                ('sub_nodes', 'subscribers'),
                ('pub_nodes', 'publishers'),
                ('srv_nodes', 'services'),
                ('scl_nodes', 'service_clients'),
            ]:
                for node_field in getattr(mod, field_name).values():
                    topic = node_field.topic
                    if topic in ['/tf_static', '/tf', '/clock']:
                        continue
                    assert topic[0] == '/'
                    if topic not in topic_infos:
                        topic_infos[topic] = TopicInfo(name=topic)
                    topic_info = topic_infos[topic]
                    getattr(topic_info, list_name).append(node_name)

        self.recurse(cb)

        print(f'digraph {{', file=f)
        print(f'rankdir="LR"', file=f)

        node_style = f'shape="box3d"'
        topic_style = f'shape="note"'
        service_style = f'shape="note",style="dashed"'

        nodes = set(sum([sum(
            [info.pub_nodes, info.sub_nodes, info.srv_nodes, info.scl_nodes],
            start=[]) for info in topic_infos.values()], start=[]))
        items = [(node, 'node') for node in nodes] + list(topic_infos.items())
        items = list(sorted(items, key=lambda elem: elem[0]))
        items.append(('/node', 'node'))

        namespace = '/'
        for name, info in items:
            # close namespace/cluster:
            while name[:len(namespace)] != namespace:
                namespace = namespace[:namespace[:-1].rindex('/') + 1]
                assert namespace[-1] == '/'
                print(f'}}', file=f)
            short_name = name[len(namespace):]
            # open namespace/cluster:
            while '/' in short_name:
                pos = short_name.index('/')
                namespace = f'{namespace}{short_name[:pos + 1]}'
                assert namespace[-1] == '/'
                short_name = name[len(namespace):]
                cluster_name = namespace.replace("/", "_").strip('_')
                print(f'subgraph cluster_{cluster_name} {{', file=f)
                print(f'label="{namespace}"', file=f)

            if info == 'node':
                # draw node
                print(f'"{name}" [label="{short_name}",{node_style}]', file=f)
                continue

            # draw topic, either normal topic or service topic
            is_topic = info.pub_nodes or info.sub_nodes
            is_service = info.srv_nodes or info.scl_nodes
            assert is_topic != is_service

            if is_topic:
                color = 'black' if info.pub_nodes else 'red'
                print(f'"T{name}" [{topic_style},color="{color}",'
                      f'label="{short_name}"]', file=f)
                edge_style = f'color="{color}"'
                for pub_node in info.pub_nodes:
                    print(f'"{pub_node}" -> "T{name}" [{edge_style}]', file=f)
                for sub_node in info.sub_nodes:
                    print(f'"T{name}" -> "{sub_node}" [{edge_style}]', file=f)
            else:
                color = 'black' if info.srv_nodes else 'red'
                print(f'"T{name}" [{service_style},color="{color}",'
                      f'label="{short_name}"]', file=f)
                edge_style = f'color="{color}",style="dashed"'
                for srv_node in info.srv_nodes:
                    print(f'"{srv_node}" -> "T{name}" [{edge_style}]', file=f)
                for scl_node in info.scl_nodes:
                    print(f'"T{name}" -> "{scl_node}" [{edge_style}]', file=f)

        print(f'"topic" [{topic_style}]', file=f)
        print(f'"service" [{service_style}]', file=f)

        print('}', file=f)



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


def check_override_valid(key: str, overrides: OverridesT):
    if key in overrides:
        raise LaunchConfigException(f'Override for "{key}" already exists!')
    if len([pattern for pattern, _ in overrides.items() if matches(key, pattern)]) > 0:
        raise LaunchConfigException(
            f'Override for "{key}" is already matched by previous glob patterns!')
    if len([pattern for pattern, _ in overrides.items() if matches(pattern, key)]) > 0:
        raise LaunchConfigException(
            f'Glob pattern override for "{key}" matches existing patterns!')


class OverrideSetter:
    def __init__(self, overrides: OverridesT, path: List[str]):
        object.__setattr__(self, '_overrides', overrides)
        object.__setattr__(self, '_path', path)

    def __setattr__(self, key: str, value: Any):
        key = '.'.join(self._getpath() + [key])
        overrides = self._getoverrides()
        check_override_valid(key, overrides)
        self._getoverrides()[key] = (value, 0, [])

    def __getattr__(self, key: str):
        return OverrideSetter(self._getoverrides(), self._getpath() + [key])

    def _getpath(self) -> List[str]:
        return object.__getattribute__(self, '_path')

    def _getoverrides(self) -> OverridesT:
        return object.__getattribute__(self, '_overrides')


RecurseLeafFunc = Callable[[LaunchConfig, LeafLaunch, str, LaunchGroup], None]
RecurseEnterFunc = Callable[[LaunchConfig,
                             LaunchGroup, str, Optional[LaunchGroup]], None]
RecurseExitFunc = Callable[[LaunchConfig,
                            LaunchGroup, str, Optional[LaunchGroup]], None]
