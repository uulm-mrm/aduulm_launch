from __future__ import annotations
from typing import Dict, Any, Tuple, List, Callable, Generator
from .types import *


class LaunchConfig:
    def __init__(self, config: LaunchConfig | None = None, path: List[str] = [], data: LaunchGroup = LaunchGroup({})):
        self.config = config if config is not None else self
        self.path = path
        self.data = data

    def __enter__(self):
        self.old_data = self.config.data
        self.config.data = self.data
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.config.data = self.old_data

    def __getattr__(self, key):
        if key in ['old_data', 'config', 'data', 'path']:
            return super().__getattribute__(key)
        if key not in self.data.modules:
            raise AttributeError(key)
        val = self.data.modules[key]
        if isinstance(val, LaunchGroup):
            return self.group(key)
        return val

    def resolve_topic(self, topic: str):
        return self.path + [topic]

    def group(self, group_name: str):
        # check if group exists, otherwise add group
        data = self.data
        if group_name in data.modules:
            group = data.modules[group_name]
            if not isinstance(group, LaunchGroup):
                raise Exception(f'trying to enter group {group_name}, but it already exists and is not a group! Type: {type(group)}')
        else:
            group = LaunchGroup({})
            data.modules[group_name] = group
        return LaunchConfig(self.config, self.path + [group_name], group)

    def add(self, name: str, child: LeafLaunch):
        if name in self.data.modules:
            raise Exception(f'trying to add {name} to current group, but something already exists there! {self.data.modules[name]}')
        self.data.modules[name] = child

    def add_sublaunch_ros(self, name: str, package_name: str, launch_filename: str, **kwargs: Any):
        self.add(name, SubLaunchROS_(package_name, launch_filename, args=kwargs))

    def include(self, desc: SubLaunchImport_):
        pass

    def exec_sublaunch(self, func: ConfigGeneratorFunc, **args: Any):
        func(self, **args)

    def exec_sublaunch_lazy(self, name: str, func: ConfigGeneratorFunc, **args: Any):
        self.add(name, SubLaunchExecLazy_(func, args=args))

    def add_node(self, name: str, package_name: str, executable_name: str, remappings: Dict[str, Topic] = {}, **parameters: Any):
        self.add(name, RunNode_(package_name, executable_name, remappings=remappings, parameters=parameters))

    def add_executable(self, name: str, executable_name: str, **args: Any):
        self.add(name, Executable_(executable_name, args=args))

    def enable_all(self):
        self.set_enabled(True)

    def disable_all(self):
        self.set_enabled(False)

    def _recurse(self, mod: AnyLaunch, func: Callable[[LeafLaunch], None]):
        if isinstance(mod, LaunchGroup):
            for mod in mod.modules.values():
                self._recurse(mod, func)
        else:
            func(mod)

    def set_enabled(self, enabled: bool):
        def set_enabled(leaf: LeafLaunch):
            leaf.enabled = enabled
        self._recurse(self.data, set_enabled)

    def items(self) -> Generator[Tuple[str, LaunchConfig | LeafLaunch], None, None]:
        for key, mod in self.data.modules.items():
            if isinstance(mod, LaunchGroup):
                yield key, self.group(key)
            else:
                yield key, mod

    def __iter__(self) -> Generator[str, None, None]:
        yield from self.data.modules.__iter__()

    def values(self) -> Generator[LaunchConfig | LeafLaunch, None, None]:
        for _, mod in self.items():
            yield mod


ConfigGeneratorFunc = Callable[[LaunchConfig, ...], None]
