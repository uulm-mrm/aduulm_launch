from aduulm_launch_lib_py import LaunchConfig, LaunchGroup, AnyLaunch, SubLaunchROS_, Executable_, RunNode_, SubLaunchExecLazy_
from typing import Any, List, Optional

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import PushRosNamespace

def convert_config_to_ros2_launch(config: LaunchConfig):
    config.evaluate()

    modules = []

    def recurse(config: LaunchConfig, mod: AnyLaunch, name: Optional[str], modules: List[Any]):
        print(mod, name, modules)
        if isinstance(mod, LaunchGroup):
            if name is not None:
                config = config.group(name)
            group_modules = []
            for key, mod in mod.modules.items():
                recurse(config, mod, key, group_modules)
            if name is not None:
                modules.extend([
                    GroupAction(actions=[
                        PushRosNamespace(name),
                        *modules
                    ])
                ])
            else:
                modules.extend(group_modules)
            return
        assert isinstance(name, str)
        if isinstance(mod, SubLaunchROS_):
            pass
        elif isinstance(mod, Executable_):
            pass
        elif isinstance(mod, RunNode_):
            pass
        elif isinstance(mod, SubLaunchExecLazy_):
            pass
        else:
            assert False

    modules = []
    recurse(config, config.data, None, modules)
    desc = LaunchDescription(modules)
    return desc
