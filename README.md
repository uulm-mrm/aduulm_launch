# aduulm_launch

Aduulm_launch provides a wrapper around the ROS 2 launch system which aims at simplifying the creation and usage of launch files.

## License

License: Apache 2.0

Affiliation: Institute of Measurement, Control and Microtechnology, Ulm University.

## Features

### Dataclasses for Parameters

aduulm_launch simplifies the way launch file parameters are declared and used. Instead of tedious substitutions with late evaluation, parameters
have to be declared once within a python dataclass:

```python
@dataclass
class Parameters:
  dummy_parameter: float
```

Overrides from the CLI can be specified using the common ROS syntax (`parameter:=value`) and are accepted using 
a single line of code within the general `gen_config` function:

```python
def gen_config(config: LaunchConfig, params: Parameters):
  config.insert_overrides(params)
```

After this call, the new value for each parameter has been set and can be used as any other python variable.
Moreover, the CLI interface comes with even more features which are described later on.

In case the parameters are directly forwarded to a node, aduulm_launch offers utility functions for the necessary
conversion to a dict: 

```python
from aduulm_launch_lib_py.utils import asdict_filtered

config.add_node(name='node_name',
                package='package_name',
                executable='executable_name',
                parameters=asdict_filtered(dataclass_instance))
```

In contrast to the built-in `as_dict` functionality of python, this function ignores any keys with `None` values.

### Namespace creation

Instead of creating namespaces using `PushRosNamespace`, aduulm_launch provides a more pythonic and easier readable equivalent using python's `with` statements:

```python
with config.group('namespace_name'):
  # modules within the namespace, e.g. my_other_module.gen_config(params)
```

### Topic resolution with respect to namespaces

In complex launch configurations it often happens that certain topics should be specified in a relative way with respect to 
their potentially unknown surrounding namespace. In aduulm_launch, this use case is covered via the following function:

```python
with config.group("nodes"):
  topic = config.resolve_topic("any_topic_name")
```

The function prefixes the given (relative) topic name with all nested namespaces, leading to `/nodes/any_topic_name` in the 
example from above.

### Parameter overrides via CLI

The general syntax is the same as for ROS: `name:=value`. In case the parameter expects a list, it may be necessary to wrap
the whole statement into quotation marks to prevent inference with the shell:
`"name:=[val1, val2]"`. For list of strings it is sometimes necessary to wrap the individual strings into quotation marks
to avoid interference of characters like `*` with the parser:
`'name:=["val_with_*_in_it", "val2"]'`
All parameter overrides are checked with respect to the type information specified within the dataclass. In this way,
invalid parameter overrides can be identified before the launch process itself has been started (i.e. no node has been started yet).

In complex launch files multiple instances of the same node may be combined
within different namespaces. In this case, it is possible to specify overrides for a
namespace by prefixing the parameter with the corresponding namespaces (separated by `.`):

```python
python3 launchfile.py camera.front_center.launch:=False
```

Be aware that the shown override will apply to any that are located in the namespace `/camera/front_center/` and contain a
parameter `launch`. Targeting individual nodes within the same namespace is currently not possible.

To simplify the usage further, aduulm_launch supports wildcards for namespaces. A single underscore followed by a dot (`_.`) will match one namespace, thereby allowing to address multiple nodes at the same time:
```python
python3 launchfile.py camera._.launch:=False
```

To match an arbitrary sequence of characters (including the namespace separator `.`), a double underscore (`__`) has to be used. This is equivalent to the RegEx syntax `.*`, and can be utilized, e.g., to match namespaces with any level of nesting:
```python
python3 launchfile.py __.launch:=False  # matches any parameter 'launch' in a namespace, i.e. 'camera.front_center.launch', 'camera.front_right.launch', 'lidar.launch', etc.
```

In case you want to set a parameter which appears without a surrounding namespace as well as with namespaces,
simply omit the dot after the wildcard:
```python
python3 launchfile.py __launch:=False  # matches everything from above, and also a top-level parameter 'launch'
```

Matching different variations of a namespace's name is also possible this way:
```python
python3 launchfile.py camera.front__.launch:=False  # matches both 'camera.front_center.launch' and 'camera.front_right.launch'
```
This should be used with care, however, as the above example would also match parameters 'launch' in a sub-level namespace like 'camera.front_center.image_correction.launch'.

Another option to pass parameter overrides to the launchfile is via the `-o` option:
```python
python3 launchfile.py -o a_lot_of_parameter_overrides.yaml 
```
Within the YAML file the `=` of the `:=` has to be omitted. Following
the above examples the file content could for instance look like this:
```python
camera._.launch: False
lidar._.launch: True
```
Storing the overriddes in a file is especially useful with a growing number 
of overridden arguments.

### Parameter Overview

In complex systems it is often hard to remember the name of all available parameters. Thus, aduulm_launch
features the possibility to obtain a full list of all parameters included within the current launch configuration with the 
`-l` flag:

```python
python3 launchfile.py -l
```

For finding individual parameters, we recommend forwarding the output via pipes to CLI utilities like `grep`.

### Compatibility with ROS launch files

In order to support a smooth transition towards aduulm_launch, functionality for including native ROS launch files is included
via the `add_sublaunch_ros` function. 
Please keep in mind that a mixture of aduulm_launch and ROS launchfiles may not be able to support all the above-mentioned convenience functions.


## Usage Example

Note: this example is solely intended for displaying the overall structure. More examples can be found within the ros2/examples 
folder.

```python
  from dataclasses import dataclass, field
  from typing import Optional
  from aduulm_launch_lib_py import LaunchConfig
  from aduulm_launch_lib_py.utils import asdict_filtered, dataclass_from_yaml
  from aduulm_launch_py import execute_with_params

  @dataclass
  class AcquisitionParams:
    launch: bool = True
    # assumes that the node already provides a default for the port 
    # which can still be overridden via CLI
    port: Optional[int] = None
    
  @dataclass 
  class ProcessingParams:
    launch: bool = True
    threshold: float = 0.5
  
  @dataclass
  class Parameters:
    # artificial top level parameter which does not provide a default
    # --> value has to be specified during launch, e.g. 'top_level_param:=5.0'
    top_level_param: float
    
    # can consist of multiple other dataclasses
    acquisition: AcquisitionParams = field(default_factory=AcquisitionParams)
    processing: ProcessingParams = field(default_factory=ProcessingParams)
  
  def gen_config(config: LaunchConfig, params: Parameters):
    # this line is always required to accept overrides via CLI 
    config.insert_overrides(params)
  
    if params.acquisition.launch:
      config.add_node(
        name='acquisition',
        package='dummy_package',
        executable='acquisition_executable',
        parameters=asdict_filtered(params.acquisition)
      )

    if params.processing.launch:
      config.add_node(
        name='processing',
        package='dummy_package',
        executable='processing_executable',
        parameters=asdict_filtered(params.processing)
      )
        
    # nesting different sub-launch files is achieved by calling the respective
    # 'gen_config()' function of the module
  
  if __name__ == "__main__":
      execute_with_params(gen_config, Parameters)
  ```

## Integration with ros2_def

Some features of aduulm_launch are especially intended for the integration with the [ros2_def](https://github.com/uulm-mrm/ros2_def),
a framework for deterministic execution of ROS stacks. Amongst others this includes
functions for listing subscribers and publishers of nodes.

More documentation regarding the integration may follow in the future.

## Aduulm Recommendations

Unfold to see a list of internal recommendations which we enforce within our own launchfile code. While these
recommendations may be relevant to other users as well, they are completely independent of the package and 
can thus be ignored by any other users.

<details>
  <summary> Aduulm Recommendations </summary>

  
    
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
  
  #### Basic launch file structure
  
  Each launch file requires at least the definition of the parameter dataclass as well as a `gen_config`
  function which handles launching the nodes. The definition of publishers and subscribers is only strictly required in
  case the node should be used with the [determinstic execution framework](https://github.com/uulm-mrm/ros2_def). However, since it is linked to the topic remappings
  defining the publisher and subscriber behaviour is recommended anyway.
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
      # and for usage with the deterministic execution framework)
  
  
  def gen_config_standalone(config: LaunchConfig, params: StandaloneParameters):
      config.insert_overrides(params)
      
      # loading configuration parameters from a file is optional, otherwise all
      # parameters without defaults have to be specified via overrides
      assert params.config_file_path is not None and params.config_file_path.exists()
  
      node_params = dataclass_from_yaml(Parameters, params.config_file_path)
  
      gen_config(config, node_params)
  
  
  if __name__ == "__main__":
      execute_with_params(gen_config_standalone, Parameters)
  ```
  </details>
</details>

<details>
<summary> Aduulm Repository Metadata </summary>
### Aduulm Repository Metadata

- last updated: 07/2024
- name: aduulm_launch
- category: tooling
- maintainers: Johannes Kopp
- license: Apache 2.0
- HW dependencies: none
</details>
