import unittest
from aduulm_launch_lib_py.launch_config import LaunchConfig, SaferDict
from aduulm_launch_lib_py.types import LaunchGroup, Node
from typing import Any


class LaunchConfigTest(unittest.TestCase):
    def _add_test_node(self, config: LaunchConfig):
        node_args: dict[str, Any] = dict(
            package_name='test_package', executable_name='test_executable')
        params: dict[str, Any] = dict(arg1='value1')
        config.add_node(name='test_node', **node_args, parameters=params)
        return node_args, params

    def test_simple(self):
        config = LaunchConfig()
        with config.group('test'):
            node_args, params = self._add_test_node(config)

        ref = LaunchGroup(modules=SaferDict(
            test=LaunchGroup(modules=SaferDict(
                test_node=Node(
                    **node_args, parameters=SaferDict(**params), remappings=SaferDict())
            ))
        ))
        self.assertEqual(config._getdata(), ref)

    def test_enable(self):
        config = LaunchConfig()
        with config.group('test'):
            self._add_test_node(config)
        node = config._getdata().modules['test'].modules['test_node']
        self.assertFalse(node.enabled)
        config.enable_all()
        self.assertTrue(node.enabled)
        config.disable_all()
        self.assertFalse(node.enabled)

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

    def test_chain_access(self):
        config = LaunchConfig()
        with config.group('test'):
            with config.group('test2'):
                node_args, params = self._add_test_node(config)
        self.assertEqual(len(list(config.iter_groups())), 1)
        self.assertEqual(len(list(config.group('test').iter_groups())), 1)
        self.assertEqual(
            len(list(config.group('test').group('test2').iter_groups())), 0)
        self.assertEqual(
            len(list(config.group('test').group('test2').iter_leaves())), 1)
        self.assertEqual(
            len(list(config.group('test').group('test2').items())), 1)

    def test_dot_access(self):
        config = LaunchConfig()
        with config.group('test'):
            with config.group('test2'):
                self._add_test_node(config)
        self.assertEqual(len(list(config.iter_groups())), 1)
        self.assertEqual(len(list(config.test.iter_groups())), 1)
        self.assertEqual(len(list(config.test.test2.iter_groups())), 0)
        self.assertEqual(len(list(config.test.test2.iter_leaves())), 1)
        self.assertEqual(len(list(config.test.test2.items())), 1)
        self.assertEqual(
            config.test.test2.test_node.get_node().parameters['arg1'], 'value1')
        self.assertEqual(
            config.test.test2.test_node.get_leaf().parameters['arg1'], 'value1')
