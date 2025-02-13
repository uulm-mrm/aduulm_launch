from dataclasses import dataclass
from aduulm_launch_lib_py import LaunchConfig
from aduulm_launch_lib_py.utils import asdict_filtered
from aduulm_launch_py import execute_with_params


@dataclass
class Params:
    background_r: int = 0
    background_g: int = 255
    background_b: int = 0

    chatter_py_ns: str = "chatter/py/ns"


def gen_config(config: LaunchConfig, params: Params):
    # this line is always required to accept overrides via CLI
    config.insert_overrides(params)

    # since the parameters are stored within a 'normal' python dataclass further interaction with them is straight
    # forward, e.g. printing for debugging purposes
    # print("Parameters: ", params)

    # include another launch file
    config.add_sublaunch_ros(name='sublaunch',
                             package='demo_nodes_cpp',
                             launchfile='launch/topics/talker_listener_launch.py')

    # include a Python launch file in the chatter_py_ns namespace
    with config.group(params.chatter_py_ns):
        config.add_sublaunch_ros(name='sublaunch_with_namespace',
                                 package='demo_nodes_cpp',
                                 launchfile='launch/topics/talker_listener_launch.py')

    # start a turtlesim_node in the turtlesim1 namespace
    with config.group("turtlesim1"):
        config.add_node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        )

    # start another turtlesim_node in the turtlesim2 namespace
    # and use args to set parameters
    with config.group("turtlesim2"):
        config.add_node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            # Note: chatter_py_ns is included here, if not desired filter the dict or restructure the params as
            #       composition. Any parameters with 'none' as value are filtered and not forwarded to the node.
            parameters=asdict_filtered(params)
        )

    # perform remap so both turtles listen to the same command topic
    forward_turtlesim_commands_to_second_turtlesim_node = config.add_node(
        package='turtlesim',
        executable='mimic',
        name='mimic',
    )

    config.add_publisher(forward_turtlesim_commands_to_second_turtlesim_node,
                         name='/input/pose', topic='/turtlesim1/turtle1/pose')
    config.add_subscriber(forward_turtlesim_commands_to_second_turtlesim_node,
                          name='/output/cmd_vel', topic='/turtlesim2/turtle1/cmd_vel')


if __name__ == "__main__":
    execute_with_params(gen_config, Params)
