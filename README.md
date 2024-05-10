# aduulm_launch

### License

The source code is not officially released and is only for internal use.

Affiliation: Institute of Measurement, Control and Microtechnology, Ulm University.

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

### Aduulm Repository Metadata

- last updated: 05/2024
- name: aduulm_launch
- category: tooling
- maintainers: Jan Strohbeck, Johannes Kopp
- license: internal use only
- HW dependencies: none

### Tips for Usage

#### Order of overrides

In order to establish a common behaviour all launch files should apply overrides in the same order (from first to last):

0.) defaults specified within the node  
1.) defaults specified in the parameter dataclass  
2.) values loaded from a YAML file  
3.) overrides obtained via command line

If a node provides a valid default for a parameter, the corresponding entry in the 
launch file dataclass should be typed as `Optional[...]` and defaulted to `None`.
For parameters without a valid default in the node, the launch file dataclass should not 
provide a default (especially not `None`). This ensures that while launching, the parameter is either provided
in a YAML configuration file or manually via the command line.

#### Loading configuration files

```python
from aduulm_launch_lib_py.utils import dataclass_from_yaml

@dataclass(slots=True)
class Parameters:
    dummy_parameter: float

dataclass_instance = dataclass_from_yaml(Parameters, path_to_config_file)
```

By not specifying a default value for the exemplary `dummy_parameter`, it is ensured
that the config file has to include an override for this parameter.

Per default, the utility function will complain about parameters which are read from the config file but
not part of the dataclass definition. To disable this behaviour, simply add `ignore_unknown=True` to the function call.

#### Parameter overrides via CLI

The general syntax is the same as for ROS: `name:=value`. In case the parameter expects as list, it may be necessary to wrap
the whole statement into quotation marks to prevent inference with the shell:
`"name:=[val1, val2]"`.
It has to be noted that overriding nested dataclass members is currently not possible.

Overrides can also be specified for individual nodes in complex launch structures using their namespace:
```python
python3 live.py platform_id:=ul1500 camera.front_center.launch:=False
```
In this case, it is also possible to use wildcards (`_`) for individual namespaces:
```python
python3 live.py platform_id:=ul1500 camera._.launch:=False
```

#### Passing dataclasses to ROS nodes

Since ROS nodes expect parameters as a dictionary, a conversion from the internally used 
dataclass is necessary. To this end, a modified version of the built-in `as_dict` function can be imported via
```from aduulm_launch_lib_py.utils import asdict_filtered```. 
In contrast to the built-in function, it will filter out all parameters which are `None`.

Passing the parameters on to the node is then as simple as:

```python
config.add_node(name='node_name',
                package='package_name',
                executable='executable_name',
                parameters=asdict_filtered(dataclass_instance))
```

#### Basic launch file structure

Each launch file requires at least the definition of the parameter dataclass as well as a `gen_config` 
function which handles launching the nodes. The definition of publishers and subscribers is only strictly required in
case the node should be used with the orchestrator. However, since it is linked to the topic remappings
defining the publisher and subscriber behaviour is recommended anyways.
Furthermore, providing standalone functionality is often convenient in case individual nodes
have to be debugged / tested in a larger context.

<details>
  <summary> Example </summary>

```python
from dataclasses import dataclass
from pathlib import Path
from aduulm_launch_lib_py import LaunchConfig
from aduulm_launch_lib_py.utils import asdict_filtered, dataclass_from_yaml
from aduulm_launch_py import execute_with_params

@dataclass(slots=True)
class Parameters:
    # all kinds of parameters relevant for the node
    pass

@dataclass(slots=True)
class StandaloneParameters:
    # may also be derived from Parameters
    config_file_path: Path


def gen_config(config: LaunchConfig, params: Parameters):
    config.insert_overrides(params)

    node = config.add_node(
        name='dummy_node',
        package='dummy_package',
        executable='dummy_executable',
        handle_lifecycle=False,
        parameters=asdict_filtered(params)
    )

    # add publishers and subscribers of the node here (required to remap topics 
    # and for usage with the orchestrator)


def gen_config_standalone(config: LaunchConfig, params: StandaloneParameters):
    config.insert_overrides(params)
    assert params.config_file_path is not None and params.config_file_path.exists()

    node_params = dataclass_from_yaml(Parameters, params.config_file_path)

    gen_config(config, node_params)


if __name__ == "__main__":
    execute_with_params(gen_config_standalone, Parameters)
```
</details>
