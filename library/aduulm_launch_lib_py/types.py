from __future__ import annotations
from dataclasses import dataclass, field
from typing import Any, List
from typing import TypeVar, Generic, Callable

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


@dataclass(slots=True)
class Node:
    package: str
    executable: str
    parameters: SaferDict[str, Any] = field(default_factory=SaferDict)
    remappings: SaferDict[str, Topic] = field(default_factory=SaferDict)
    args: List[str] = field(default_factory=list)
    output: str = 'screen'
    emulate_tty: bool = True
    xterm: bool = False
    gdb: bool = False


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
