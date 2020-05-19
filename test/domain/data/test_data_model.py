#!/usr/bin/env python
# -*- coding: utf-8 -*-
import logging
import sys
import unittest
from os import path
import rospy

sys.path.append(
    path.dirname(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
    + "/scripts"
)
from error import MyException
from domain.data.model import DataId, DataConnectionId

PKG = "skyway"


class TestDataModel(unittest.TestCase):
    def test_create_data_id_success(self):
        data_id = DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211")
        data_id2 = DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211")
        self.assertEqual(data_id.id(), u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211")
        self.assertEqual(data_id, data_id2)

    def test_create_data_id_success_different_ids(self):
        data_id = DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a210")
        data_id2 = DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211")
        self.assertEqual(data_id.id(), u"da-50a32bab-b3d9-4913-8e20-f79c90a6a210")
        self.assertNotEqual(data_id, data_id2)

    def test_create_data_id_fail_due_to_length(self):
        with self.assertRaises(MyException):
            _data_id = DataId(u"da-50a32bab-b3d9-4913-8e20")

    def test_create_data_id_fail_due_to_data_type(self):
        with self.assertRaises(MyException):
            _data_id = DataId("da-50a32bab-b3d9-4913-8e20-f79c90a6a210")

    def test_create_data_id_fail_due_to_wrong_prefix(self):
        with self.assertRaises(MyException):
            _data_id = DataId(u"dc-50a32bab-b3d9-4913-8e20-f79c90a6a210")

    def test_create_data_connection_id_success(self):
        data_connection_id = DataConnectionId(
            u"dc-50a32bab-b3d9-4913-8e20-f79c90a6a211"
        )
        data_connection_id2 = DataConnectionId(
            u"dc-50a32bab-b3d9-4913-8e20-f79c90a6a211"
        )
        self.assertEqual(
            data_connection_id.id(), u"dc-50a32bab-b3d9-4913-8e20-f79c90a6a211"
        )
        self.assertEqual(data_connection_id, data_connection_id2)

    def test_create_data_connection_id_different_ids(self):
        data_connection_id = DataConnectionId(
            u"dc-50a32bab-b3d9-4913-8e20-f79c90a6a211"
        )
        data_connection_id2 = DataConnectionId(
            u"dc-50a32bab-b3d9-4913-8e20-f79c90a6a212"
        )
        self.assertEqual(
            data_connection_id.id(), u"dc-50a32bab-b3d9-4913-8e20-f79c90a6a211"
        )
        self.assertNotEqual(data_connection_id, data_connection_id2)

    def test_create_data_connection_id_fail_due_to_length(self):
        with self.assertRaises(MyException):
            _data_connection_id = DataConnectionId(u"dc-50a32bab-b3d9-4913-8e20")

    def test_create_data_connection_id_fail_due_to_data_type(self):
        with self.assertRaises(MyException):
            _data_connection_id = DataConnectionId(
                "dc-50a32bab-b3d9-4913-8e20-f79c90a6a211"
            )

    def test_create_data_connection_id_fail_due_to_wrong_prefix(self):
        with self.assertRaises(MyException):
            _data_connection_id = DataConnectionId(
                u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211"
            )


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "data_model", TestDataModel)
