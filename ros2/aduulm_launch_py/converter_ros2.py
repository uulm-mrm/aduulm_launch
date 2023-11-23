from aduulm_launch_lib_py import LaunchConfig, LaunchGroup, AnyLaunch, SubLaunchROS, Executable, Node
from typing import Any, List, Optional

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node as ROSNode
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from aduulm_tools_python.launch_utils import get_package_share_directory
import os


def convert_config_to_ros2_launch(config: LaunchConfig):
    config.evaluate()
    modules = []

    def recurse(config: LaunchConfig, mod: AnyLaunch, name: Optional[str], modules: List[Any]):
        if isinstance(mod, LaunchGroup):
            if name is not None:
                config = config.group(name)
            group_modules = []
            for key, child in mod.modules.items():
                recurse(config, child, key, group_modules)
            if name is not None:
                arg_name = 'launch_' + '__'.join(config._getpath())
                modules.extend([
                    DeclareLaunchArgument(arg_name, default_value='true'),
                    GroupAction(
                        actions=[
                            PushRosNamespace(name),
                            *group_modules
                        ],
                        # scoped=False,
                        # forwarding=False,
                        condition=IfCondition(LaunchConfiguration(arg_name))
                    )
                ])
            else:
                modules.extend(group_modules)
            return
        assert isinstance(name, str)
        arg_name = 'launch_' + '__'.join(config._getpath() + [name])
        arg = DeclareLaunchArgument(arg_name, default_value=str(mod.enabled))
        modules.append(arg)

        def handle_common(mod: Executable | Node) -> dict[str, Any]:
            prefix = ''
            if mod.xterm:
                prefix += 'xterm -maximized -bg black -fg white -hold -sl 100000 -e '
            if mod.gdb:
                prefix += 'gdb -ex run --args'
            return dict(
                name=name,
                output=mod.output,
                emulate_tty=mod.emulate_tty,
                prefix=prefix,
                condition=IfCondition(LaunchConfiguration(arg_name))
            )
        if isinstance(mod, SubLaunchROS):
            desc = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory(mod.package_name),
                        'launch',
                        mod.launch_filename
                    )
                ),
                launch_arguments=mod.args.items(),
                condition=IfCondition(LaunchConfiguration(arg_name))
            )
            modules.append(desc)
        elif isinstance(mod, Executable):
            desc = ExecuteProcess(
                cmd=[mod.executable_name, *mod.args],
                **handle_common(mod)
            )
            modules.append(desc)
        elif isinstance(mod, Node):
            desc = ROSNode(
                package=mod.package_name,
                executable=mod.executable_name,
                parameters=[mod.parameters.todict()],
                remappings=mod.remappings.items(),
                **handle_common(mod)
            )
            modules.append(desc)
        else:
            # SubLaunchExecLazy_ must not occur here, because we called evaluate() beforehand!
            assert False

    modules = []
    recurse(config, config._getdata(), None, modules)
    desc = LaunchDescription(modules)
    return desc
