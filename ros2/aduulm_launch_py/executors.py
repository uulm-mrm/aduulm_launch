from aduulm_launch_lib_py import LaunchConfig
from .executor_ros2 import execute_config_with_ros2_launch
import sys
from typing import Callable, ParamSpec, Concatenate, List
from launch.actions.include_launch_description import LaunchDescriptionEntity


P = ParamSpec('P')


def execute(gen_config: Callable[Concatenate[LaunchConfig, P], None], _exit=True, _extra_ros2_modules: List[LaunchDescriptionEntity] = [], *args: P.args, **kwargs: P.kwargs):
    config = LaunchConfig()
    config.parse_argv()

    gen_config(config, *args, **kwargs)
    config.check_overrides_counts()

    ret = execute_config_with_ros2_launch(
        config, extra_modules=_extra_ros2_modules)
    if _exit:
        sys.exit(ret)
    return ret
