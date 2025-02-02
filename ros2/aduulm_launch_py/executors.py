import argparse
import pathlib
import sys
from dataclasses import is_dataclass
from inspect import signature
from typing import Callable, ParamSpec, Concatenate, List, TypeVar, Optional

from launch.actions.include_launch_description import LaunchDescriptionEntity

from aduulm_launch_lib_py import LaunchConfig, LaunchConfigException
from .executor_ros2 import execute_config_with_ros2_launch


P = ParamSpec('P')
PT = TypeVar('PT')


def call_config_with_params(config: LaunchConfig, gen_config: Callable[Concatenate[LaunchConfig, PT, P], None], *args: P.args, **kwargs: P.kwargs):
    dataclass_params = [(i, name, param.annotation) for i, (name, param) in enumerate(
        signature(gen_config).parameters.items()) if is_dataclass(param.annotation)]
    assert len(dataclass_params) == 1
    sys_args = _parse_args(config)
    arg_pos, name, params_cls = dataclass_params[0]
    assert arg_pos == 1
    assert is_dataclass(params_cls) and isinstance(params_cls, type)
    params = config.instantiate_dataclass_from_overrides(params_cls)
    assert name not in kwargs
    gen_config(config, params, *args, **kwargs)
    config.check_overrides_counts()
    return config, sys_args


def execute_with_params(gen_config: Callable[Concatenate[LaunchConfig, PT, P], None],
                        _exit=True, _extra_ros2_modules: List[LaunchDescriptionEntity] = [], *args: P.args, _initial_config: Optional[LaunchConfig] = None, **kwargs: P.kwargs):
    config = LaunchConfig() if _initial_config is None else _initial_config
    config, sys_args = call_config_with_params(
        config, gen_config, *args, **kwargs)
    return _execute(config, sys_args=sys_args, _exit=_exit, _extra_ros2_modules=_extra_ros2_modules)


def execute(gen_config: Callable[Concatenate[LaunchConfig, P], None],
            _exit=True, _extra_ros2_modules: List[LaunchDescriptionEntity] = [], *args: P.args, **kwargs: P.kwargs):
    config = LaunchConfig()
    sys_args = _parse_args(config)
    gen_config(config, *args, **kwargs)
    return _execute(config, sys_args=sys_args, _exit=_exit, _extra_ros2_modules=_extra_ros2_modules)


def _parse_args(config: LaunchConfig):
    parser = argparse.ArgumentParser()
    parser.add_argument('args', type=str, nargs='*', default=[])
    parser.add_argument('-p', '--params', type=str, nargs='+', default=[])
    parser.add_argument('-d', '--debug', action='store_true')
    parser.add_argument('-o', '--overrides_file', type=str)
    parser.add_argument('-l', '--list_params', action='store_true')
    parser.add_argument('-g', '--export_graphviz', action='store_true')
    sys_args = parser.parse_args()
    if sys_args.overrides_file:
        overrides_file = pathlib.Path(sys_args.overrides_file)
        assert overrides_file.exists(), overrides_file
    config.parse_args(sys_args.args, sys_args.params, sys_args.overrides_file)
    return sys_args


def _execute(config: LaunchConfig, sys_args, _exit=True, _extra_ros2_modules: List[LaunchDescriptionEntity] = []):
    if sys_args.export_graphviz:
        gv_path = pathlib.Path('/tmp') / 'launch.gv'
        with open(gv_path, 'w') as f:
            config.generate_topic_graphviz(f)
        print(f'exported graphviz file to {str(gv_path)}')
        print(f'now launch "dot -Tpdf {str(gv_path)} -o {str(gv_path)}.pdf"')
        exit(0)
    if sys_args.list_params:
        list_params(config)
        sys.exit(0)
    ret = execute_config_with_ros2_launch(
        config, debug=sys_args.debug, extra_modules=_extra_ros2_modules)
    if _exit:
        sys.exit(ret)
    return ret


def list_params(config: LaunchConfig):
    print(f'the following params are available to override:')
    for type1, fields in config._getavail_overrides():
        print(f'from {type1}:')
        for field in fields:
            print(f'  {field.name}: {repr(field.value)} ({field.field_type})')
