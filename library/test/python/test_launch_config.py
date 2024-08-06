import io
import pathlib
import unittest
from aduulm_launch_lib_py.launch_config import LaunchConfig, SaferDict, \
    LaunchGroup, Node, LaunchConfigException, OverridableField
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


@dataclass
class _TestNested:
    a: _TestParameters2
    b: _TestParameters3


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

    def test_nested_override(self):
        params = _TestNested(a=_TestParameters2(
            arg2='val'), b=_TestParameters3(arg3='val2'))
        config = LaunchConfig()
        config.overrides().test.test2.a.optional_arg = 'value1'
        config.overrides().test.test2.b.optional_arg = 'value2'
        with config.group('test'):
            with config.group('test2'):
                config.insert_overrides(params)
        self.assertEqual(params.a.optional_arg, 'value1')
        self.assertEqual(params.b.optional_arg, 'value2')
        config.check_overrides_counts()

    def test_nested_override_from_argv(self):
        config = LaunchConfig()
        config.parse_args(['test.test2.a.optional_arg:=value1',
                          'test.test2.b.optional_arg:=value2'], [])
        with config.group('test'):
            with config.group('test2'):
                a = _TestParameters2(
                    arg2='val')
                b = _TestParameters3(
                    arg3='val2')
                params = _TestNested(a=a, b=b)
                config.insert_overrides(params)
        config.check_overrides_counts()
        self.assertEqual(params.a.optional_arg, 'value1')
        self.assertEqual(params.b.optional_arg, 'value2')

    def test_dataclass_init_from_nested_override_from_argv(self):
        config = LaunchConfig()
        config.parse_args(['a.arg2:=val', 'a.optional_arg:=value1',
                          'b.arg3:=val2', 'b.optional_arg:=value2'], [])
        params = config.instantiate_dataclass_from_overrides(_TestNested)
        config.check_overrides_counts()
        self.assertEqual(params.a.arg2, 'val')
        self.assertEqual(params.a.optional_arg, 'value1')
        self.assertEqual(params.b.arg3, 'val2')
        self.assertEqual(params.b.optional_arg, 'value2')

    def test_override_from_yaml(self):
        overrides_file = pathlib.Path(__file__).parent / 'test_overrides.yaml'
        config = LaunchConfig()
        config.parse_args([], [], overrides_file)
        with config.group('test'):
            with config.group('test2'):
                params = _TestParameters(
                    required_arg='val', optional_arg1='other_val')
                config.insert_overrides(params)
                self._add_test_node(config)
        self.assertEqual(params.optional_arg2, 'preset')
        config.check_overrides_counts()

    def test_avail_overrides(self):
        config = LaunchConfig()
        config.parse_args([], [])
        config.overrides().test.test2.optional_arg2 = 'foo'
        with config.group('test'):
            with config.group('test2'):
                params = _TestParameters(
                    required_arg='val', optional_arg1='other_val')
                config.insert_overrides(params)
                self._add_test_node(config)
        self.assertEqual(
            config._getavail_overrides(),
            [(_TestParameters, [
                OverridableField(
                    name='test.test2.required_arg', field_type=str,
                    default_value=None, value='val'),
                OverridableField(
                    name='test.test2.optional_arg1', field_type=str,
                    default_value='val1', value='other_val'),
                OverridableField(
                    name='test.test2.optional_arg2', field_type=str,
                    default_value='val2', value='foo')])])

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

    def test_graphviz_generation(self):
        config = LaunchConfig()
        with config.group('test'):
            _, _, node = self._add_test_node(config)
            config.add_subscriber(node, name='sub1', topic='/ns/topic1')
            config.add_subscriber(node, name='sub2', topic='/ns/topic2')
            config.add_publisher(node, name='pub1', topic='/ns/topic1')
            config.add_service_client(node, name='client1',
                                      topic='/ns/service1')
            config.add_service_client(node, name='client2',
                                      topic='/ns/service2')
            config.add_service(node, name='srv1', topic='/ns/service1')
        f = io.StringIO()
        config.generate_topic_graphviz(f)
        self.assertEqual(
            f.getvalue(),
            'digraph {\n'
            'rankdir="LR"\n'
            'subgraph cluster_test {\n'
            'label="/test/"\n'
            '"/test/test_node" [label="test_node",shape="box3d"]\n'
            '}\n'
            '"/node" [label="node",shape="box3d"]\n'
            '"topic" [shape="note"]\n'
            '"service" [shape="note",style="dashed"]\n'
            '"T/ns/service1" [shape="note",style="dashed",color="black",label="/ns/service1"]\n'
            '"/test/test_node" -> "T/ns/service1" [color="black",style="dashed"]\n'
            '"T/ns/service1" -> "/test/test_node" [color="black",style="dashed"]\n'
            '"T/ns/service2" [shape="note",style="dashed",color="red",label="/ns/service2"]\n'
            '"T/ns/service2" -> "/test/test_node" [color="red",style="dashed"]\n'
            '"T/ns/topic1" [shape="note",color="black",label="/ns/topic1"]\n'
            '"/test/test_node" -> "T/ns/topic1" [color="black"]\n'
            '"T/ns/topic1" -> "/test/test_node" [color="black"]\n'
            '"T/ns/topic2" [shape="note",color="red",label="/ns/topic2"]\n'
            '"T/ns/topic2" -> "/test/test_node" [color="red"]\n'
            '}\n')
