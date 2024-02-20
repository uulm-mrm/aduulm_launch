from typing import Callable, TypeVar, cast, Any, Iterable
from dataclasses import asdict, fields
from pathlib import Path
from aduulm_launch_lib_py import LaunchConfigException
from yaml import safe_load
from dataclasses import is_dataclass

T1 = TypeVar('T1')


def dataclass_from_yaml(params_cls: Callable[..., T1], yaml_path: Path, ignore_unknown=False):
    with open(yaml_path, "r") as f:
        config = safe_load(f)

    def convert(val: dict[str, Any], cls: Callable[..., Any]):
        for key, v in list(val.items()):
            field_dataclass = None
            for field in fields(cast(Any, cls)):
                if field.name == key:
                    if is_dataclass(field.type):
                        field_dataclass = field.type
                    break
            else:
                if not ignore_unknown:
                    raise LaunchConfigException(
                        f"{key} was specified with value {v} but the dataclass {cls} does not contain a field with that name!")
                del val[key]
                continue
            if field_dataclass is not None:
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
