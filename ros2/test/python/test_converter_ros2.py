from aduulm_launch_lib_py import LaunchConfig
from aduulm_launch_py import convert_config_to_ros2_launch
from typing import Any
import unittest
from launch import LaunchDescription, LaunchContext
from launch.actions import GroupAction, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import PushRosNamespace, Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


class ConverterRos2Test(unittest.TestCase):
    def test_simple(self):
        config = LaunchConfig()
        node_args: dict[str, Any] = dict(
            package='test_package', executable='test_executable')
        params: dict[str, Any] = dict(arg1='value1')
        with config.group('test'):
            config.add_node(name='test_node', **node_args, parameters=params)
        res = convert_config_to_ros2_launch(config)

        context = LaunchContext()
        nodes = res.visit(context)
        assert (nodes is not None)
        self.assertEqual(len(nodes), 2)
        ga = nodes[1]
        assert isinstance(ga, GroupAction)
        ga_entities = ga._GroupAction__actions
        assert (ga_entities is not None)
        ga_pushes = [n for n in ga_entities if isinstance(n, PushRosNamespace)]
        ga_nodes = [n for n in ga_entities if isinstance(n, Node)]
        self.assertEqual(len(ga_pushes), 1)
        self.assertEqual(len(ga_nodes), 1)
        ga_nodes[0]._perform_substitutions(context)
        self.assertEqual(ga_nodes[0].node_package, node_args['package'])
        self.assertEqual(ga_nodes[0].node_executable,
                         node_args['executable'])
        self.assertTrue(ga_nodes[0].node_name.endswith(f'/test_node'))
        # cannot simply test all attributes, sadly

    def test_include_ros(self):
        config = LaunchConfig()
        include_args: dict[str, Any] = dict(
            package='test_package', launchfile='test_launch')
        args: dict[str, Any] = dict(arg1='value1')
        config.add_sublaunch_ros(name='sublaunch', **include_args, args=args)
        res = convert_config_to_ros2_launch(config)

        context = LaunchContext()
        nodes = res.visit(context)
        assert (nodes is not None)
        self.assertEqual(len(nodes), 2)
        inc_nodes = [n for n in nodes if isinstance(
            n, IncludeLaunchDescription)]
        self.assertEqual(len(inc_nodes), 1)
        source = inc_nodes[0].launch_description_source
        assert (isinstance(source, PythonLaunchDescriptionSource))

    def test_execute_process(self):
        config = LaunchConfig()
        proc_args: dict[str, Any] = dict(executable='test_executable')
        args: list[str] = ["--arg1", 'value1']
        config.add_executable(name='executable', **proc_args, args=args)
        res = convert_config_to_ros2_launch(config)

        context = LaunchContext()
        nodes = res.visit(context)
        assert (nodes is not None)
        self.assertEqual(len(nodes), 2)
        proc_nodes = [n for n in nodes if isinstance(n, ExecuteProcess)]
        self.assertEqual(len(proc_nodes), 1)
        self.assertIsNotNone(proc_nodes[0].name)
