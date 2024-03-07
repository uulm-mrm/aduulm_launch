from typing import Callable, TypeVar, cast, Any, get_args, get_origin, Union, Iterable, Dict
from dataclasses import asdict, fields
from pathlib import Path
from aduulm_launch_lib_py import LaunchConfigException
from yaml import safe_load
from dataclasses import is_dataclass

T1 = TypeVar('T1')


def dataclass_from_yaml(params_cls: Callable[..., T1], yaml_path: Path, ignore_unknown=False):
    with open(yaml_path, "r") as f:
        config = safe_load(f)
    return dataclass_from_dict(params_cls, config, ignore_unknown=ignore_unknown)


def dataclass_from_dict(params_cls: Callable[..., T1], config: Dict[str, Any], ignore_unknown=False):
    def check_is_list(t):
        if get_origin(t) is Union:
            t = get_args(t)[0]
        if get_origin(t) is list:
            return True, get_args(t)[0]
        return False, None

    def check_is_dict(t):
        if get_origin(t) is Union:
            t = get_args(t)[0]
        if get_origin(t) is dict:
            return True, get_args(t)[1]
        return False, None

    def convert(val: Dict[str, Any], cls: Callable[..., Any]):
        if val is None:
            val = {}
        for key, v in list(val.items()):
            field_dataclass = None
            is_list = False
            is_dict = False
            for field in fields(cast(Any, cls)):
                if field.name == key:
                    is_list, element_t_list = check_is_list(field.type)
                    is_dict, element_t_dict = check_is_dict(field.type)
                    if is_dataclass(field.type):
                        field_dataclass = field.type
                    elif is_list:
                        if is_dataclass(element_t_list):
                            field_dataclass = element_t_list
                    elif is_dict:
                        if is_dataclass(element_t_dict):
                            field_dataclass = element_t_dict
                    break
            else:
                if not ignore_unknown:
                    raise LaunchConfigException(
                        f"{key} was specified with value {v} but the dataclass {cls} does not contain a field with that name!")
                del val[key]
                continue
            assert not (is_list and is_dict)
            if is_list and field_dataclass is not None:
                val[key] = [convert(v, field_dataclass) for v in val[key]]
            elif is_dict and field_dataclass is not None:
                val[key] = {k: convert(v, field_dataclass)
                            for k, v in val[key].items()}
            elif field_dataclass is not None:
                val[key] = convert(val[key], field_dataclass)
        try:
            return cls(**val)
        except TypeError as e:
            raise LaunchConfigException(
                f"Instantiation of {cls} from {val} constructed from {yaml_path} failed. Maybe a parameter is missing in the YAML file or has an incorrect type? Check the error message above.") from e
    assert is_dataclass(params_cls) and isinstance(params_cls, type)
    return cast(T1, convert(config, params_cls))


def asdict_filtered(obj: Callable[..., T1]):
    return asdict(obj, dict_factory=lambda x: {k: v for (k, v) in x if v is not None and (not isinstance(v, Iterable) or len(v) > 0)})
