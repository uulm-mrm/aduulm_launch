from aduulm_launch_lib_py import LaunchConfig
from .executor_ros2 import execute_config_with_ros2_launch
import sys
from typing import Callable, ParamSpec, Concatenate


P = ParamSpec('P')


def execute(gen_config: Callable[Concatenate[LaunchConfig, P], None], _exit=True, *args: P.args, **kwargs: P.kwargs):
    config = LaunchConfig()
    config.parse_argv()

    gen_config(config, *args, **kwargs)
    config.check_overrides_counts()

    ret = execute_config_with_ros2_launch(config)
    if _exit:
        sys.exit(ret)
    return ret
