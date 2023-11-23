from __future__ import annotations
from typing import Dict, Any, Tuple, List, Callable, Generator, Optional, Concatenate, ParamSpec
from .types import *


class LaunchConfig:
    def __init__(self, config: LaunchConfig | None = None, path: List[str] = None, data: Optional[LaunchGroup] = None):
        self._config = config if config is not None else self
        self._path = path if path is not None else []
        self._data = data if data is not None else LaunchGroup()
        self._val: Optional[LeafLaunch] = None

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

    def __enter__(self):
        self._old_data = self._getconfig()._getdata()
        self._getconfig()._data = self._data
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._getconfig()._data = self._old_data

    def resolve_topic(self, topic: str):
        return self._getpath() + [topic]

    def group(self, group_name: str):
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
        return LaunchConfig(self._getconfig(), self._path + [group_name], group)

    def add(self, name: str, child: LeafLaunch):
        if name in self._getdata().modules:
            raise Exception(
                f'trying to add {name} to current group, but something already exists there! {self._data.modules[name]}')
        self._getdata().modules.add(name, child)

    def add_sublaunch_ros(self, name: str, package_name: str, launch_filename: str, args: Dict[str, Any] = {}):
        self.add(name, SubLaunchROS_(package_name,
                 launch_filename, args=SaferDict(**args)))

    def exec_sublaunch(self, func: ConfigGeneratorFunc, **args: Any):
        func(self, **args)

    def exec_sublaunch_lazy(self, name: str, func: ConfigGeneratorFunc, **args: Any):
        self.add(name, SubLaunchExecLazy_(func, args=SaferDict(**args)))

    def add_node(self, name: str, package_name: str, executable_name: str, remappings: Dict[str, Topic] = {},
                 parameters: Dict[str, Any] = {}, output: str = 'screen', emulate_tty: bool = True):
        self.add(name, RunNode_(package_name, executable_name, remappings=SaferDict(**remappings),
                                parameters=SaferDict(**parameters), output=output, emulate_tty=emulate_tty))

    def evaluate(self):
        def do_evaluate(config: LaunchConfig, mod: LeafLaunch, name: str, parent: LaunchGroup):
            if isinstance(mod, SubLaunchExecLazy_):
                del parent.modules[name]
                mod.func(config, **mod.args)
                # TODO set submodules enabled based on sublaunch enabled
        self.recurse(do_evaluate)

    def add_executable(self, name: str, executable_name: str, args: List[str] = [], output: str = 'screen', emulate_tty: bool = True):
        self.add(name, Executable_(executable_name, args=args[:],
                 output=output, emulate_tty=emulate_tty))

    def enable_all(self):
        self.set_enabled(True)

    def disable_all(self):
        self.set_enabled(False)

    def recurse(self, cb_leaf: RecurseLeafFunc, cb_enter: Optional[RecurseEnterFunc] = None, cb_exit:
                Optional[RecurseExitFunc] = None):
        self._do_recurse(self, self._getdata(), None, None,
                         cb_leaf, cb_enter, cb_exit)

    def _do_recurse(self, config: LaunchConfig, mod: AnyLaunch, name: Optional[str], parent: Optional[LaunchGroup],
                    cb_leaf: RecurseLeafFunc, cb_enter: Optional[RecurseEnterFunc] = None, cb_exit: Optional[RecurseExitFunc] = None):
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

    def set_enabled(self, enabled: bool):
        def set_enabled(config: LaunchConfig, mod: LeafLaunch, name: str, parent: LaunchGroup):
            mod.enabled = enabled
        self.recurse(set_enabled)

    def items(self) -> Generator[Tuple[str, LaunchConfig | LeafLaunch], None, None]:
        for key, mod in self._getdata().modules.items():
            if isinstance(mod, LaunchGroup):
                yield key, self.group(key)
            else:
                yield key, mod

    def iter_leaves(self) -> Generator[Tuple[str, LeafLaunch], None, None]:
        for key, mod in self._getdata().modules.items():
            if not isinstance(mod, LaunchGroup):
                yield key, mod

    def iter_groups(self) -> Generator[Tuple[str, LaunchConfig], None, None]:
        for key, mod in self._getdata().modules.items():
            if not isinstance(mod, LaunchGroup):
                yield key, self.group(key)

    def __iter__(self) -> Generator[str, None, None]:
        yield from self._getdata().modules.__iter__()

    def values(self) -> Generator[LaunchConfig | LeafLaunch, None, None]:
        for _, mod in self.items():
            yield mod

    def __repr__(self):
        return repr(self._getdata())

    def __str__(self):
        return str(self._getdata())

    def __getattr__(self, key):
        if key not in self._getconfig()._getdata().modules:
            raise AttributeError(key)
        val = self._getconfig()._getdata().modules[key]
        if isinstance(val, LaunchGroup):
            return LaunchConfig(self._getconfig().group(key))
        self._val = val
        return self

    def get_leaf(self) -> LeafLaunch:
        val = self._getval()
        assert val is not None
        return val

    def get_node(self):
        val = self._getval()
        assert val is not None
        assert isinstance(val, RunNode_)
        return val

    def get_group(self):
        assert self._getval() is None
        return self._getconfig()

    def get_executable(self):
        val = self._getval()
        assert val is not None
        assert isinstance(val, Executable_)
        return val

    def get_sublaunch_ros(self):
        val = self._getval()
        assert val is not None
        assert isinstance(val, SubLaunchROS_)
        return val

    def get_sublaunch_exec_lazy(self):
        val = self._getval()
        assert val is not None
        assert isinstance(val, SubLaunchExecLazy_)
        return val


RecurseLeafFunc = Callable[[LaunchConfig, LeafLaunch, str, LaunchGroup], None]
RecurseEnterFunc = Callable[[LaunchConfig,
                             LaunchGroup, str, Optional[LaunchGroup]], None]
RecurseExitFunc = Callable[[LaunchConfig,
                            LaunchGroup, str, Optional[LaunchGroup]], None]
P = ParamSpec('P')
ConfigGeneratorFunc = Callable[Concatenate[LaunchConfig, P], None]
