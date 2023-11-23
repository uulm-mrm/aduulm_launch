from __future__ import annotations
from dataclasses import dataclass, field
from typing import Dict, Any, List
from typing import TypeVar, Generic, Callable

Topic = str


@dataclass(kw_only=True, slots=True)
class Enableable:
    enabled: bool = False

    def enable(self):
        self.enabled = True

    def disable(self):
        self.enabled = False


K = TypeVar('K')
V = TypeVar('V')


class SaferDict(Generic[K, V]):
    def __init__(self, **kwargs: V):
        self._data: Dict[K, V] = dict(**kwargs)

    # only allow __setitem__ if key already exists
    def __setitem__(self, key: K, value: V):
        if key not in self._data:
            raise Exception(
                f"Could not replace argument {key} because it does not exist!")
        self._data[key] = value

    # only allow add if key does not already exist
    def add(self, key: K, value: V):
        if key in self._data:
            raise Exception(
                f"Could not add argument {key} because it already exists!")
        self._data[key] = value
        return value

    # The following methods are proxies for dict methods.
    # Maybe these could be replaced by inheriting from dict, but the typing seems to require Python >= 3.12
    def todict(self):
        return self._data

    def __getitem__(self, key: K) -> V:
        return self._data[key]

    def __iter__(self):
        return self._data.__iter__()

    def keys(self):
        return self._data.keys()

    def values(self):
        return self._data.values()

    def items(self):
        return self._data.items()

    def __repr__(self):
        return repr(self._data)

    def __str__(self):
        return str(self._data)

    def __delitem__(self, key: K):
        del self._data[key]

    def __eq__(self, other: Any):
        if isinstance(other, SaferDict):
            return self._data == other._data
        return self._data == other


@dataclass(slots=True)
class Executable_(Enableable):
    executable_name: str
    args: List[str] = field(default_factory=list)
    output: str = 'screen'
    emulate_tty: bool = True
    xterm: bool = False
    gdb: bool = False


@dataclass(slots=True)
class RunNode_(Enableable):
    package_name: str
    executable_name: str
    parameters: SaferDict[str, Any] = field(default_factory=SaferDict)
    remappings: SaferDict[str, Topic] = field(default_factory=SaferDict)
    output: str = 'screen'
    emulate_tty: bool = True
    xterm: bool = False
    gdb: bool = False


@dataclass(slots=True)
class SubLaunchROS_(Enableable):
    package_name: str
    launch_filename: str
    args: SaferDict[str, Any] = field(default_factory=SaferDict)


ConfigGeneratorFuncAny = Callable[..., None]


@dataclass(slots=True)
class SubLaunchExecLazy_(Enableable):
    func: ConfigGeneratorFuncAny
    args: SaferDict[str, Any] = field(default_factory=SaferDict)


SubLaunch_ = SubLaunchROS_ | SubLaunchExecLazy_


@dataclass
class LaunchGroup:
    modules: SaferDict[str, 'Executable_ | RunNode_ | SubLaunch_ | LaunchGroup'] = field(
        default_factory=SaferDict)


def Executable(executable_name: str, args: List[Any] = [], output: str = 'screen', emulate_tty: bool = True):
    return Executable_(executable_name, args=args, output=output, emulate_tty=emulate_tty)


def RunNode(package_name: str, executable_name: str, remappings: Dict[str, Topic] = {}, parameters: Dict[str, Any] = {}, output: str = 'screen', emulate_tty: bool = True):
    return RunNode_(package_name, executable_name, remappings=SaferDict(**remappings), parameters=SaferDict(**parameters), output=output, emulate_tty=emulate_tty)


def SubLaunchROS(package_name: str, launch_filename: str, **args: Any):
    return SubLaunchROS_(package_name, launch_filename, args=SaferDict(**args))


def SubLaunchExecLazy(func: ConfigGeneratorFuncAny, **args: Any):
    return SubLaunchExecLazy_(func, args=SaferDict(**args))


LeafLaunch = Executable_ | RunNode_ | SubLaunch_
AnyLaunch = LeafLaunch | LaunchGroup
