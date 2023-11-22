import unittest
from aduulm_launch_lib_py.launch_config import LaunchConfig, SaferDict
from aduulm_launch_lib_py.types import LaunchGroup, RunNode_
from typing import Any


class LaunchConfigTest(unittest.TestCase):
    def test_simple(self):
        config = LaunchConfig()
        node_args: dict[str, Any] = dict(package_name='test_package', executable_name='test_executable')
        params: dict[str, Any] = dict(arg1='value1')
        with config.group('test'):
            config.add_node(name='test_node', **node_args, **params)
        ref = LaunchGroup(modules=SaferDict(
            test=LaunchGroup(modules=SaferDict(
                test_node=RunNode_(**node_args, parameters=SaferDict(**params), remappings=SaferDict())
            ))
        ))
        self.assertEqual(config.data, ref)
