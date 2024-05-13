from launch.actions import RegisterEventHandler, EmitEvent
from aduulm_launch_lib_py import LaunchConfig, LaunchGroup, AnyLaunch, SubLaunchROS, Executable, Node, LogLevel
from typing import Any, List, Optional, cast, Literal, Union

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction, ExecuteProcess, Shutdown, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.actions import EmitEvent
from launch.actions.include_launch_description import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace, Node as ROSNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.actions import LifecycleNode
from launch.events import matches_action
from aduulm_tools_python.launch_utils import get_package_share_directory
from lifecycle_msgs.msg import Transition
import os


log_level_map = {
    LogLevel.Debug: "debug",
    LogLevel.Info: "info",
    LogLevel.Warning: "warn",
    LogLevel.Error: "error",
    LogLevel.Fatal: "none",
}


def handle_activation(node_desc: LifecycleNode):
    handler_configure = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=node_desc,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(
                            node_desc),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
    )
    handler_activate = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=node_desc,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(
                            node_desc),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
    )
    return [node_desc, handler_configure, handler_activate]


def convert_config_to_ros2_launch(config: LaunchConfig, extra_modules: List[LaunchDescriptionEntity] = []):
    modules: List[LaunchDescriptionEntity] = []

    def recurse(config: LaunchConfig, mod: AnyLaunch, name: Optional[str], modules: List[LaunchDescriptionEntity]):
        if isinstance(mod, LaunchGroup):
            if name is not None:
                config = config.group(name)
            group_modules: List[LaunchDescriptionEntity] = []
            group_modules += mod.ros2_entities
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
                assert not mod.valgrind
                prefix += 'gdb -ex run --args '
            if mod.valgrind:
                prefix += 'valgrind '
            prefix = prefix.strip()
            return dict(
                name=(name if mod.set_name else None),
                output=mod.output,
                emulate_tty=mod.emulate_tty,
                prefix=prefix,
            )
        if isinstance(mod, SubLaunchROS):
            desc = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory(mod.package),
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
            args = mod.args[:]
            if len(mod.loggers) > 0:
                args.append('--ros-args')
                for logger in mod.loggers:
                    args += ['--log-level',
                             f'{logger.node_name}:={log_level_map[logger.log_level]}']
            if isinstance(mod.handle_lifecycle, bool):
                cls = ROSNode if not mod.handle_lifecycle else LifecycleNode
                handle_lifecycle = ['configuration',
                                    'activation'] if mod.handle_lifecycle else []
            else:
                assert not ('activation' in mod.handle_lifecycle and
                            'configuration' not in mod.handle_lifecycle), f'Activation of {mod.package} without configuration'
                handle_lifecycle = mod.handle_lifecycle
                cls = LifecycleNode
            desc = cls(
                package=mod.package,
                executable=mod.executable,
                parameters=[mod.parameters.toparamdict()],
                remappings=mod.get_remappings().items(),
                arguments=args,
                **handle_common(mod),
                on_exit=None if not mod.required else Shutdown(),
                namespace='',
                additional_env=mod.additional_env.tostrdict()
            )
            modules.append(desc)

            if 'configuration' in handle_lifecycle:
                handler_configure = RegisterEventHandler(
                    event_handler=OnProcessStart(
                        target_action=desc,
                        on_start=[
                            EmitEvent(
                                event=ChangeState(
                                    lifecycle_node_matcher=matches_action(
                                        desc),
                                    transition_id=Transition.TRANSITION_CONFIGURE,
                                ),
                            ),
                        ],
                    ),
                )
                modules += [handler_configure]

            if 'activation' in handle_lifecycle:
                handler_activate = RegisterEventHandler(
                    event_handler=OnStateTransition(
                        target_lifecycle_node=cast(LifecycleNode, desc),
                        start_state='configuring',
                        goal_state='inactive',
                        entities=[
                            EmitEvent(
                                event=ChangeState(
                                    lifecycle_node_matcher=matches_action(
                                        desc),
                                    transition_id=Transition.TRANSITION_ACTIVATE,
                                ),
                            ),
                        ],
                    ),
                )
                modules += [handler_activate]
        else:
            assert False

    modules = []
    modules += extra_modules

    recurse(config, config._getdata(), None, modules)
    desc = LaunchDescription(modules)
    return desc
