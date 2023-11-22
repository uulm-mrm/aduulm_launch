from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, Any

@dataclass
class Executable_:
    executable_name: str
    args: Dict[str, Any]

    enabled: bool = False

Topic = str

@dataclass
class RunNode_:
    package_name: str
    executable_name: str
    parameters: Dict[str, Any]
    remappings: Dict[str, Topic]

    enabled: bool = False

@dataclass
class SubLaunchROS_:
    package_name: str
    launch_filename: str
    args: Dict[str, Any]

    enabled: bool = False

@dataclass
class SubLaunchExecLazy_:
    func: 'ConfigGeneratorFunc'
    args: Dict[str, Any]

    enabled: bool = False

SubLaunch_ = SubLaunchROS_ | SubLaunchExecLazy_

@dataclass
class LaunchGroup:
    modules: Dict[str, 'Executable_ | RunNode_ | SubLaunch_ | LaunchGroup']

def Executable(executable_name: str, **kwargs):
    return Executable_(executable_name, args=kwargs)

def RunNode(package_name: str, executable_name: str, remappings: Dict[str, Topic]  ={}, **parameters: Any):
    return RunNode_(package_name, executable_name, remappings=remappings, parameters=parameters)

def SubLaunchROS(package_name: str, launch_filename: str, **args: Any):
    return SubLaunchROS_(package_name, launch_filename, args=args)

def SubLaunchImport(package_name: str, launch_filename: str, **args: Any):
    return SubLaunchImport_(package_name, launch_filename, args=args)

def SubLaunchImportLazy(package_name: str, launch_filename: str, **args: Any):
    return SubLaunchImportLazy_(package_name, launch_filename, args=args)

LeafLaunch = Executable_ | RunNode_ | SubLaunch_
AnyLaunch = LeafLaunch | LaunchGroup

def AduulmNode(**kwargs: Any):
    return None
