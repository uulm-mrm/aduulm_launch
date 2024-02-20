import unittest
from typing import List, Dict
from dataclasses import dataclass, field
from aduulm_launch_lib_py.utils import dataclass_from_dict


@dataclass
class DummyStruct:
    test_bool: bool
    test_str: str
    test_float: float
    test_int: int
    test_default: str = "kkk"


@dataclass
class DummyStructWithList:
    test_list: List[str] = field(default_factory=list)


@dataclass
class DummyInnerStruct:
    a: int


@dataclass
class DummyStructWithListOfStruct:
    test_list: List[DummyInnerStruct]


@dataclass
class DummyStructWithDict:
    test_dict: Dict[str, int]


@dataclass
class DummyStructWithDictOfStruct:
    test_dict: Dict[str, DummyInnerStruct]


class UtilsTest(unittest.TestCase):
    def test_load_simple(self):
        data = {
            "test_bool": True,
            "test_str": "test",
            "test_float": -1.2,
            "test_int": 2
        }
        struct = dataclass_from_dict(DummyStruct, data)
        self.assertEqual(struct, DummyStruct(**data))

    def test_load_list(self):
        data = {
            "test_list": ["xxx", "kkk"]
        }
        struct = dataclass_from_dict(DummyStructWithList, data)
        self.assertEqual(struct, DummyStructWithList(**data))

    def test_load_list_complex(self):
        data = {
            "test_list": [
                {"a": 1},
                {"a": 2},
            ]
        }
        struct = dataclass_from_dict(DummyStructWithListOfStruct, data)
        self.assertEqual(struct, DummyStructWithListOfStruct(
            test_list=[DummyInnerStruct(a=1), DummyInnerStruct(a=2)]))

    def test_load_dict(self):
        data = {
            "test_dict": {
                "a": 1,
                "b": 2,
            }
        }
        struct = dataclass_from_dict(DummyStructWithDict, data)
        self.assertEqual(struct, DummyStructWithDict(
            test_dict={"a": 1, "b": 2}))

    def test_load_dict_complex(self):
        data = {
            "test_dict": {
                "a": {"a": 1},
                "b": {"a": 2},
            }
        }
        struct = dataclass_from_dict(DummyStructWithDictOfStruct, data)
        self.assertEqual(struct, DummyStructWithDictOfStruct(
            test_dict={"a": DummyInnerStruct(a=1), "b": DummyInnerStruct(a=2)}))
