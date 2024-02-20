import unittest
from aduulm_launch_lib_py.launch_config import LaunchConfig, SaferDict, LaunchGroup, Node, LaunchConfigException
from typing import Any, cast
from dataclasses import dataclass


@dataclass
class _TestParameters:
    required_arg: str
    optional_arg1: str = 'val1'
    optional_arg2: str = 'val2'


@dataclass
class _TestParameters2:
    arg2: str
    optional_arg: str = 'val'


@dataclass
class _TestParameters3:
    arg3: str
    optional_arg: str = 'val'


class LaunchConfigTest(unittest.TestCase):
    def _add_test_node(self, config: LaunchConfig, params: dict[str, Any] = dict(arg1='value1')):
        node_args: dict[str, Any] = dict(
            package='test_package', executable='test_executable')
        node = config.add_node(name='test_node', **
                               node_args, parameters=params)
        return node_args, params, node

    def test_simple(self):
        config = LaunchConfig()
        with config.group('test'):
            node_args, params, _ = self._add_test_node(config)

        ref = LaunchGroup(modules=SaferDict(
            test=LaunchGroup(modules=SaferDict(
                test_node=Node(
                    **node_args, parameters=SaferDict(**params))
            ))
        ))
        self.assertEqual(config._getdata(), ref)

    def test_resolve_topic(self):
        config = LaunchConfig()
        topic = config.resolve_topic('detections')
        self.assertEqual(topic, '/detections')
        ns1 = config.resolve_namespace()
        with config.group('test'):
            ns2 = config.resolve_namespace()
            topic2 = config.resolve_topic('detections')
            with config.group('test2'):
                ns3 = config.resolve_namespace()
                topic3 = config.resolve_topic('detections')
        self.assertEqual(topic2, '/test/detections')
        self.assertEqual(topic3, '/test/test2/detections')
        self.assertEqual(ns1, '')
        self.assertEqual(ns2, '/test')
        self.assertEqual(ns3, '/test/test2')

    def test_chain_access(self):
        config = LaunchConfig()
        with config.group('test'):
            with config.group('test2'):
                self._add_test_node(config)
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
            cast(Node, config.test.test2.test_node.get_leaf()).parameters['arg1'], 'value1')

    def test_dot_access_toplevel(self):
        config = LaunchConfig()
        self._add_test_node(config)
        self.assertEqual(
            config.test_node.get_node().parameters['arg1'], 'value1')
        self.assertEqual(len(list(config.iter_leaves())), 1)

    def test_set_overrides(self):
        config = LaunchConfig()
        config.overrides().test.test2.launch_test_node = True

        def func():
            config.overrides().test.test2.launch_test_node = True
        self.assertRaises(LaunchConfigException, func)
        self.assertEqual(config._getoverrides()[
                         'test.test2.launch_test_node'], (True, 0, []))

    def test_get_overrides_toplevel(self):
        params = _TestParameters(required_arg='val', optional_arg1='other_val')
        config = LaunchConfig()
        config.overrides().optional_arg2 = 'preset'
        config.insert_overrides(params)
        self.assertEqual(params.required_arg, 'val')
        self.assertEqual(params.optional_arg1, 'other_val')
        self.assertEqual(params.optional_arg2, 'preset')
        config.check_overrides_counts()

    def _test_overrides(self, params: Any):
        config = LaunchConfig()
        config.overrides().test.test2.optional_arg2 = 'preset'
        with config.group('test'):
            with config.group('test2'):
                config.insert_overrides(params)
        return config

    def test_get_overrides(self):
        params = _TestParameters(required_arg='val', optional_arg1='other_val')
        config = self._test_overrides(params)
        self.assertEqual(params.required_arg, 'val')
        self.assertEqual(params.optional_arg1, 'other_val')
        self.assertEqual(params.optional_arg2, 'preset')
        config.check_overrides_counts()

    def test_throw_overrides_twice(self):
        params = _TestParameters(
            required_arg='val', optional_arg1='other_val', optional_arg2='other_val2')
        self.assertEqual(params.required_arg, 'val')
        self.assertEqual(params.optional_arg1, 'other_val')
        self.assertEqual(params.optional_arg2, 'other_val2')
        self._test_overrides(params)

    def test_throw_overrides_not_applied(self):
        params = _TestParameters2(arg2='val')
        config = self._test_overrides(params)
        self.assertRaises(LaunchConfigException, config.check_overrides_counts)

    def _test_wildcard(self, config: LaunchConfig):
        with config.group('test'):
            with config.group('test2'):
                params2 = _TestParameters2(arg2='val')
                config.insert_overrides(params2)
            with config.group('test3'):
                params3 = _TestParameters3(arg3='val')
                config.insert_overrides(params3)
        return params2, params3

    def test_wildcard(self):
        config = LaunchConfig()
        config.overrides().test._.optional_arg = 'preset'
        params2, params3 = self._test_wildcard(config)
        self.assertEqual(params2.optional_arg, 'preset')
        self.assertEqual(params3.optional_arg, 'preset')
        config.check_overrides_counts()

    def test_wildcard2(self):
        config = LaunchConfig()
        config.overrides().__.optional_arg = 'preset'
        params2, params3 = self._test_wildcard(config)
        self.assertEqual(params2.optional_arg, 'preset')
        self.assertEqual(params3.optional_arg, 'preset')
        config.check_overrides_counts()

    def test_wildcard3(self):
        config = LaunchConfig()
        config.overrides()._.optional_arg = 'preset'
        params2, params3 = self._test_wildcard(config)
        self.assertEqual(params2.optional_arg, 'val')
        self.assertEqual(params3.optional_arg, 'val')
        self.assertRaises(LaunchConfigException, config.check_overrides_counts)

    def _test_param_override(self, config: LaunchConfig):
        with config.group('test'):
            with config.group('test2'):
                self._add_test_node(config)

    def test_param_override(self):
        config = LaunchConfig()
        config.param_overrides().test.test2.test_node.use_sim_time = True
        self._test_param_override(config)
        self.assertEqual(
            config.test.test2.test_node.get_node().parameters['use_sim_time'], True)
        config.check_overrides_counts()

    def test_param_override_wildcard(self):
        config = LaunchConfig()
        config.param_overrides().__.use_sim_time = True
        self._test_param_override(config)
        self.assertEqual(
            config.test.test2.test_node.get_node().parameters['use_sim_time'], True)
        config.check_overrides_counts()

    def test_param_override_throws(self):
        config = LaunchConfig()
        config.param_overrides().test.use_sim_time = True
        self._test_param_override(config)
        self.assertRaises(LaunchConfigException, config.check_overrides_counts)

    def test_param_override_throws2(self):
        config = LaunchConfig()
        config.param_overrides().__.use_sim_time = True
        self._test_param_override(config)
        self.assertRaises(LaunchConfigException, lambda: config.test.test2.test_node.get_node(
        ).parameters.add('use_sim_time', True))

    def test_param_override_throws3(self):
        config = LaunchConfig()
        config.param_overrides().__.use_sim_time = True
        self._test_param_override(config)
        with config.group('test'):
            with config.group('test2'):
                self.assertRaises(LaunchConfigException, lambda: self._add_test_node(
                    config, params={'use_sim_time': True}))

    def test_override_from_argv(self):
        config = LaunchConfig()
        config.parse_args(['test.test2.optional_arg2:=preset'], [
                          'test.test2.test_node.use_sim_time:=True'])
        with config.group('test'):
            with config.group('test2'):
                params = _TestParameters(
                    required_arg='val', optional_arg1='other_val')
                config.insert_overrides(params)
                self._add_test_node(config)
        self.assertEqual(params.optional_arg2, 'preset')
        self.assertEqual(
            config.test.test2.test_node.get_node().parameters['use_sim_time'], True)
        config.check_overrides_counts()

    def test_remappings(self):
        config = LaunchConfig()
        with config.group('test'):
            with config.group('test2'):
                _, _, node = self._add_test_node(config)
                config.add_subscriber(node, name='sub1', topic='/ns/mysub')
                config.add_publisher(node, name='pub1', topic='/ns/mypub')
                config.add_service_client(
                    node, name='client1', topic='/ns/myclient')
                config.add_service(node, name='srv1', topic='/ns/mysrv')
                config.add_timer(node, period=0.1)
        self.assertEqual(node.get_remappings(), {
            'sub1': '/ns/mysub',
            'pub1': '/ns/mypub',
            'client1': '/ns/myclient',
            'srv1': '/ns/mysrv',
        })
