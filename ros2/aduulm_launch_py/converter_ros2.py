from aduulm_launch_lib_py import LaunchConfig, LaunchGroup, AnyLaunch, SubLaunchROS, Executable, Node
from typing import Any, List, Optional

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess
from launch.actions.include_launch_description import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node as ROSNode
from aduulm_tools_python.launch_utils import get_package_share_directory
import os


def convert_config_to_ros2_launch(config: LaunchConfig, extra_modules: List[LaunchDescriptionEntity] = []):
    modules = []

    def recurse(config: LaunchConfig, mod: AnyLaunch, name: Optional[str], modules: List[Any]):
        if isinstance(mod, LaunchGroup):
            if name is not None:
                config = config.group(name)
            group_modules = []
            for key, child in mod.modules.items():
                recurse(config, child, key, group_modules)
            if name is not None:
                modules.extend([
                    GroupAction(
                        actions=[
                            PushRosNamespace(name),
                            *group_modules
                        ],
                        # scoped=False,
                        # forwarding=False,
                    )
                ])
            else:
                modules.extend(group_modules)
            return
        assert isinstance(name, str)

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
            )
        if isinstance(mod, SubLaunchROS):
            desc = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory(mod.package),
                        'launch',
                        mod.launchfile
                    )
                ),
                launch_arguments=mod.args.tostrdict().items(),
            )
            modules.append(desc)
        elif isinstance(mod, Executable):
            desc = ExecuteProcess(
                cmd=[mod.executable, *mod.args],
                **handle_common(mod)
            )
            modules.append(desc)
        elif isinstance(mod, Node):
            desc = ROSNode(
                package=mod.package,
                executable=mod.executable,
                parameters=[mod.parameters.toparamdict()],
                remappings=mod.get_remappings().items(),
                arguments=mod.args,
                **handle_common(mod)
            )
            modules.append(desc)
        else:
            assert False

    modules = []
    modules += extra_modules

    recurse(config, config._getdata(), None, modules)
    desc = LaunchDescription(modules)
    return desc
