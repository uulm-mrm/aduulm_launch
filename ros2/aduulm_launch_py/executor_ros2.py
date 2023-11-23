from aduulm_launch_lib_py import LaunchConfig
from .converter_ros2 import convert_config_to_ros2_launch
from launch import LaunchDescription

from typing import Optional

import launch
import sys


def execute_config_with_ros2_launch(config: LaunchConfig, debug: bool = False, noninteractive: Optional[bool] = None):
    launch_description = convert_config_to_ros2_launch(config)

    return run_launchdescription(launch_description, debug=debug, noninteractive=noninteractive)


def run_launchdescription(launch_description: LaunchDescription, debug: bool = False, noninteractive: Optional[bool] = None):
    launch_service = launch.LaunchService(
        argv=sys.argv,
        noninteractive=(not sys.stdin.isatty()
                        ) if noninteractive is None else noninteractive,
        debug=debug)

    launch_service.include_launch_description(launch_description)

    ret = launch_service.run()
    return ret


def print_launchdescription(launch_description: LaunchDescription):
    print(launch.LaunchIntrospector().format_launch_description(launch_description))
