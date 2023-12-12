from aduulm_launch_lib_py import LaunchConfig, LaunchConfigException
from .executor_ros2 import execute_config_with_ros2_launch
import sys
from typing import Callable, ParamSpec, Concatenate, List, TypeVar, cast
from launch.actions.include_launch_description import LaunchDescriptionEntity
from dataclasses import is_dataclass
import argparse


P = ParamSpec('P')
PT = TypeVar('PT')


def execute_with_params(gen_config: Callable[Concatenate[LaunchConfig, PT, P], None],
                        params_cls: Callable[..., PT], _exit=True, _extra_ros2_modules: List[LaunchDescriptionEntity] = [], *args: P.args, **kwargs: P.kwargs):
    config = LaunchConfig()
    sys_args = _parse_args(config)
    assert is_dataclass(params_cls) and isinstance(params_cls, type)
    overrides = config.get_overrides(params_cls)
    try:
        params = params_cls(**{k: v for k, _, v, _ in overrides})
    except TypeError as e:
        raise LaunchConfigException(
            f'Could not construct instance of dataclass type {params_cls}! Probably the class has required fields but no override was provided!') from e
    for k, _, _, _ in overrides:
        config.inc_override_count(k, params)
    gen_config(config, cast(PT, params), *args, **kwargs)
    return _execute(config, _debug=sys_args.debug, _exit=_exit, _extra_ros2_modules=_extra_ros2_modules)


def execute(gen_config: Callable[Concatenate[LaunchConfig, P], None],
            _exit=True, _extra_ros2_modules: List[LaunchDescriptionEntity] = [], *args: P.args, **kwargs: P.kwargs):
    config = LaunchConfig()
    sys_args = _parse_args(config)
    gen_config(config, *args, **kwargs)
    return _execute(config, _debug=sys_args.debug, _exit=_exit, _extra_ros2_modules=_extra_ros2_modules)


def _parse_args(config: LaunchConfig):
    parser = argparse.ArgumentParser()
    parser.add_argument('args', type=str, nargs='*', default=[])
    parser.add_argument('-p', '--params', type=str, nargs='+', default=[])
    parser.add_argument('-d', '--debug', action='store_true')
    sys_args = parser.parse_args()
    config.parse_args(sys_args.args, sys_args.params)
    return sys_args


def _execute(config: LaunchConfig, _debug: bool = False, _exit=True, _extra_ros2_modules: List[LaunchDescriptionEntity] = []):
    config.check_overrides_counts()
    ret = execute_config_with_ros2_launch(
        config, debug=_debug, extra_modules=_extra_ros2_modules)
    if _exit:
        sys.exit(ret)
    return ret
