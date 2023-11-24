from __future__ import annotations
from typing import Dict, Any, Tuple, List, Callable, Generator, Optional, Concatenate, ParamSpec
from .types import *
from dataclasses import fields, _MISSING_TYPE, is_dataclass
from functools import lru_cache
import re
import yaml


class LaunchConfig:
    def __init__(self, config: LaunchConfig | None = None, path: Optional[List[str]] = None, data: Optional[LaunchGroup] = None):
        self._config = config if config is not None else self
        self._path = path if path is not None else []
        self._data = data if data is not None else LaunchGroup()
        self._val: Optional[LeafLaunch] = None
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
                raise Exception(
                    f'trying to enter group {group_name}, but it already exists and is not a group! Type: {type(group)}')
        else:
            group = LaunchGroup()
            data.modules.add(group_name, group)
        return LaunchConfig(self._getconfig(), self._getpath() + [group_name], group)

    def add(self, name: str, child: LeafLaunch):
        assert self._getval() is None
        if name in self._getdata().modules:
            raise Exception(
                f'trying to add {name} to current group, but something already exists there! {self._data.modules[name]}')
        self._getdata().modules.add(name, child)

    def add_sublaunch_ros(self, name: str, package: str, launchfile: str, args: Dict[str, Any] = {}):
        assert self._getval() is None
        self.add(name, SubLaunchROS(package,
                 launchfile, args=SaferDict(**args)))

    def add_node(self, name: str, package: str, executable: str, remappings: Dict[str, Topic] = {},
                 parameters: Dict[str, Any] = {}, args: List[str] = [], output: str = 'screen', emulate_tty: bool = True):
        assert self._getval() is None
        params = SaferDict(**parameters)
        self._insert_param_overrides(name, params)
        self.add(name, Node(package, executable, remappings=SaferDict(**remappings),
                            parameters=params, args=args[:], output=output, emulate_tty=emulate_tty))

    def add_executable(self, name: str, executable: str, args: List[str] = [], output: str = 'screen', emulate_tty: bool = True):
        assert self._getval() is None
        self.add(name, Executable(executable, args=args[:],
                 output=output, emulate_tty=emulate_tty))

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
        self._val = val
        return self

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
                raise Exception(
                    f'Attribute {field.name} of {params.__class__} was overridden by override {k} with value {v} but type should be {field.type}!')
            if getattr(params, field.name) != default:
                raise Exception(
                    f'Attribute {field.name} of {params.__class__} was already assigned to and can not be overridden by override {k}!')
            setattr(params, field.name, v)
            overrides[k] = (v, usage_cnt+1)

    def check_overrides_counts(self):
        self._check_overrides_counts(self._getoverrides(), 'argument')
        self._check_overrides_counts(self._getparam_overrides(), 'parameter')

    def _check_overrides_counts(self, overrides: Dict[str, Tuple[Any, int]], _type: str):
        unused_overrides = [k for k, (_, e) in overrides.items() if e == 0]
        if len(unused_overrides) != 0:
            raise Exception(
                f'The following {_type} overrides were specified but were not applied: {unused_overrides}')

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
        raise Exception(f'Override for "{key}" already exists!')
    if len([pattern for pattern, _ in overrides.items() if matches(key, pattern)]) > 0:
        raise Exception(
            f'Override for "{key}" is already matched by previous glob patterns!')
    if len([pattern for pattern, _ in overrides.items() if matches(pattern, key)]) > 0:
        raise Exception(
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
P = ParamSpec('P')
ConfigGeneratorFunc = Callable[Concatenate[LaunchConfig, P], None]
