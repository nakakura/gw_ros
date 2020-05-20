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
from domain.data.model import Socket, DataSocket, DataId, DataConnectionId

PKG = "skyway"


class TestDataModel(unittest.TestCase):
    def test_create_socket_success(self):
        socket = Socket(10000, ip_v4=u"127.0.0.1")
        socket2 = Socket(10000, ip_v4=u"127.0.0.1")
        self.assertEqual(socket.port(), 10000)
        self.assertEqual(socket.is_ip_v4(), True)
        self.assertEqual(socket.ip_v4(), u"127.0.0.1")
        self.assertEqual(socket.ip_v6(), u"")
        self.assertEqual(socket2.port(), 10000)
        self.assertEqual(socket2.is_ip_v4(), True)
        self.assertEqual(socket2.ip_v4(), u"127.0.0.1")
        self.assertEqual(socket2.ip_v6(), u"")
        self.assertEqual(socket, socket2)

    def test_create_socket_success_other_port(self):
        socket = Socket(10000, ip_v4=u"127.0.0.1")
        socket2 = Socket(10001, ip_v4=u"127.0.0.1")
        self.assertEqual(socket.port(), 10000)
        self.assertEqual(socket.is_ip_v4(), True)
        self.assertEqual(socket.ip_v4(), u"127.0.0.1")
        self.assertEqual(socket.ip_v6(), u"")
        self.assertEqual(socket2.port(), 10001)
        self.assertEqual(socket2.is_ip_v4(), True)
        self.assertEqual(socket2.ip_v4(), u"127.0.0.1")
        self.assertEqual(socket2.ip_v6(), u"")
        self.assertNotEqual(socket, socket2)

    def test_create_socket_success_other_ip(self):
        socket = Socket(10000, ip_v4=u"127.0.0.1")
        socket2 = Socket(10000, ip_v4=u"127.0.0.2")
        self.assertEqual(socket.port(), 10000)
        self.assertEqual(socket.is_ip_v4(), True)
        self.assertEqual(socket.ip_v4(), u"127.0.0.1")
        self.assertEqual(socket.ip_v6(), u"")
        self.assertEqual(socket2.port(), 10000)
        self.assertEqual(socket2.is_ip_v4(), True)
        self.assertEqual(socket2.ip_v4(), u"127.0.0.2")
        self.assertEqual(socket2.ip_v6(), u"")
        self.assertNotEqual(socket, socket2)

    def test_create_socket_success_other_stack(self):
        socket = Socket(65535, ip_v4=u"127.0.0.1")
        socket2 = Socket(65535, ip_v6=u"fe00::1")
        self.assertEqual(socket.port(), 65535)
        self.assertEqual(socket.is_ip_v4(), True)
        self.assertEqual(socket.ip_v4(), u"127.0.0.1")
        self.assertEqual(socket.ip_v6(), u"")
        self.assertEqual(socket.json(), {"port": 65535, "ip_v4": u"127.0.0.1"})
        self.assertEqual(socket2.port(), 65535)
        self.assertEqual(socket2.is_ip_v4(), False)
        self.assertEqual(socket2.ip_v4(), u"")
        self.assertEqual(socket2.ip_v6(), u"fe00::1")
        self.assertEqual(socket2.json(), {"port": 65535, "ip_v6": u"fe00::1"})
        self.assertNotEqual(socket, socket2)

    def test_create_socket_fail_due_to_port_num(self):
        with self.assertRaises(MyException):
            _socket = Socket(0, ip_v4=u"127.0.0.1")

    def test_create_socket_fail_due_to_port_num_exceed(self):
        with self.assertRaises(MyException):
            _socket = Socket(65536, ip_v4=u"127.0.0.1")

    def test_create_socket_fail_invalid_port_type(self):
        with self.assertRaises(MyException):
            _socket = Socket("0", ip_v4=u"127.0.0.1")

    def test_create_socket_fail_invalid_ip_v4_type(self):
        with self.assertRaises(MyException):
            _socket = Socket(10000, ip_v4="0")

    def test_create_socket_fail_invalid_ip_v6_type(self):
        with self.assertRaises(MyException):
            _socket = Socket(10000, ip_v6="0")

    def test_create_socket_fail_invalid_no_ip(self):
        with self.assertRaises(MyException):
            _socket = Socket(10000, ip_v4=u"")

    def test_create_data_socket_success(self):
        data_socket = DataSocket(
            u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",
            10000,
            ip_v4=u"127.0.0.1",
            ip_v6=u"",
        )
        data_socket2 = DataSocket(
            u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",
            10000,
            ip_v4=u"127.0.0.1",
            ip_v6=u"",
        )
        self.assertEqual(data_socket, data_socket2)

    def test_create_data_socket_success_other_stack(self):
        data_socket = DataSocket(
            u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",
            10000,
            ip_v4=u"127.0.0.1",
            ip_v6=u"",
        )
        data_socket2 = DataSocket(
            u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",
            10000,
            ip_v4=u"",
            ip_v6=u"fe00::1",
        )
        self.assertNotEqual(data_socket, data_socket2)

    def test_create_data_socket_success_other_port(self):
        data_socket = DataSocket(
            u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",
            10000,
            ip_v4=u"127.0.0.1",
            ip_v6=u"",
        )
        data_socket2 = DataSocket(
            u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",
            10001,
            ip_v4=u"127.0.0.1",
            ip_v6=u"",
        )
        self.assertNotEqual(data_socket, data_socket2)

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
