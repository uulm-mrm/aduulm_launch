import unittest
from aduulm_launch_lib_py.launch_config import LaunchConfig, SaferDict
from aduulm_launch_lib_py.types import LaunchGroup, Node
from typing import Any


class LaunchConfigTest(unittest.TestCase):
    def test_simple(self):
        config = LaunchConfig()
        node_args: dict[str, Any] = dict(
            package_name='test_package', executable_name='test_executable')
        params: dict[str, Any] = dict(arg1='value1')
        with config.group('test'):
            config.add_node(name='test_node', **node_args, parameters=params)
        ref = LaunchGroup(modules=SaferDict(
            test=LaunchGroup(modules=SaferDict(
                test_node=Node(
                    **node_args, parameters=SaferDict(**params), remappings=SaferDict())
            ))
        ))
        self.assertEqual(config._getdata(), ref)

    def test_resolve_topic(self):
        config = LaunchConfig()
        topic = config.resolve_topic('detections')
        self.assertEqual(topic, '/detections')
        with config.group('test'):
            topic2 = config.resolve_topic('detections')
            with config.group('test2'):
                topic3 = config.resolve_topic('detections')
        self.assertEqual(topic2, '/test/detections')
        self.assertEqual(topic3, '/test/test2/detections')
