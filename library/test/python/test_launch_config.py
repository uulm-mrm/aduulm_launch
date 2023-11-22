import unittest
from aduulm_launch_lib_py.launch_config import LaunchConfig
from aduulm_launch_lib_py.types import LaunchGroup, RunNode_


class LaunchConfigTest(unittest.TestCase):
    def test_simple(self):
        config = LaunchConfig()
        node_args = dict(package_name='test_package', executable_name='test_executable')
        params = dict(arg1='value1')
        with config.group('test'):
            config.add_node(name='test_node', **node_args, **params)
        ref = LaunchGroup(modules={
            "test": LaunchGroup(modules={
                'test_node': RunNode_(**node_args, parameters=params, remappings={})
            })
        })
        self.assertEqual(config.data, ref)
