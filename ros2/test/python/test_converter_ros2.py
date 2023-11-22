from aduulm_launch_lib_py import LaunchConfig
from aduulm_launch_py import convert_config_to_ros2_launch
from typing import Any
import unittest
from launch import LaunchDescription
from launch.actions import GroupAction


class ConverterRos2Test(unittest.TestCase):
    def test_simple(self):
        config = LaunchConfig()
        node_args: dict[str, Any] = dict(package_name='test_package', executable_name='test_executable')
        params: dict[str, Any] = dict(arg1='value1')
        with config.group('test'):
            config.add_node(name='test_node', **node_args, **params)
        res = convert_config_to_ros2_launch(config)
        self.assertEqual(len(res.entities), 1)
