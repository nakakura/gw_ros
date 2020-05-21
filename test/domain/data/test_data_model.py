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
from domain.data.model import *
from domain.common.model import PeerInfo

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

    def test_create_dcinit(self):
        dc_init1 = DcInit(
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            }
        )
        self.assertEqual(
            dc_init1.json(),
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            },
        )
        dc_init2 = DcInit(
            {
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            }
        )
        self.assertEqual(
            dc_init2.json(),
            {
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            },
        )
        dc_init3 = DcInit(
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            }
        )
        self.assertEqual(
            dc_init3.json(),
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            },
        )
        dc_init4 = DcInit(
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            }
        )
        self.assertEqual(
            dc_init4.json(),
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            },
        )
        dc_init5 = DcInit(
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "id": 0,
                "priority": "NONE",
            }
        )
        self.assertEqual(
            dc_init5.json(),
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "id": 0,
                "priority": "NONE",
            },
        )
        dc_init6 = DcInit(
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "priority": "NONE",
            }
        )
        self.assertEqual(
            dc_init6.json(),
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "priority": "NONE",
            },
        )
        dc_init7 = DcInit(
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
            }
        )
        self.assertEqual(
            dc_init7.json(),
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
            },
        )

        with self.assertRaises(MyException):
            _dc_init_err = DcInit(
                {
                    "ordered": 0,
                    "maxPacketLifeTime": 0,
                    "maxRetransmits": 0,
                    "protocol": "H264",
                    "negotiated": True,
                    "id": 0,
                    "priority": "NONE",
                }
            )
        with self.assertRaises(MyException):
            _dc_init_err = DcInit(
                {
                    "ordered": True,
                    "maxPacketLifeTime": "0",
                    "maxRetransmits": 0,
                    "protocol": "H264",
                    "negotiated": True,
                    "id": 0,
                    "priority": "NONE",
                }
            )
        with self.assertRaises(MyException):
            _dc_init_err = DcInit(
                {
                    "ordered": True,
                    "maxPacketLifeTime": 0,
                    "maxRetransmits": "0",
                    "protocol": "H264",
                    "negotiated": True,
                    "id": 0,
                    "priority": "NONE",
                }
            )
        with self.assertRaises(MyException):
            _dc_init_err = DcInit(
                {
                    "ordered": True,
                    "maxPacketLifeTime": 0,
                    "maxRetransmits": 0,
                    "protocol": 0,
                    "negotiated": True,
                    "id": 0,
                    "priority": "NONE",
                }
            )
        with self.assertRaises(MyException):
            _dc_init_err = DcInit(
                {
                    "ordered": True,
                    "maxPacketLifeTime": 0,
                    "maxRetransmits": 0,
                    "protocol": "H264",
                    "negotiated": "HOGE",
                    "id": 0,
                    "priority": "NONE",
                }
            )
        with self.assertRaises(MyException):
            _dc_init_err = DcInit(
                {
                    "ordered": True,
                    "maxPacketLifeTime": 0,
                    "maxRetransmits": 0,
                    "protocol": "H264",
                    "negotiated": True,
                    "id": "0",
                    "priority": "NONE",
                }
            )
        with self.assertRaises(MyException):
            _dc_init_err = DcInit(
                {
                    "ordered": True,
                    "maxPacketLifeTime": 0,
                    "maxRetransmits": 0,
                    "protocol": "H264",
                    "negotiated": True,
                    "id": 0,
                    "priority": 0,
                }
            )

    def test_create_connect_inner_option(self):
        inner_option = ConnectInnerOption(
            {
                "metadata": "meta message",
                "serialization": "BINARY",
                "dcInit": {
                    "ordered": True,
                    "maxPacketLifeTime": 0,
                    "maxRetransmits": 0,
                    "protocol": "H264",
                    "negotiated": True,
                    "id": 0,
                    "priority": "NONE",
                },
            },
        )
        self.assertEqual(
            inner_option.json(),
            {
                "metadata": "meta message",
                "serialization": "BINARY",
                "dcInit": {
                    "ordered": True,
                    "maxPacketLifeTime": 0,
                    "maxRetransmits": 0,
                    "protocol": "H264",
                    "negotiated": True,
                    "id": 0,
                    "priority": "NONE",
                },
            },
        )

        inner_option2 = ConnectInnerOption({"metadata": "meta message", "dcInit": {},},)
        self.assertEqual(
            inner_option2.json(), {"metadata": "meta message", "dcInit": {},},
        )

        inner_option3 = ConnectInnerOption(
            {"metadata": "meta message", "serialization": "BINARY",},
        )
        self.assertEqual(
            inner_option3.json(),
            {"metadata": "meta message", "serialization": "BINARY",},
        )

        with self.assertRaises(MyException):
            _inner_option_err = ConnectInnerOption(
                {
                    "metadata": 0,
                    "serialization": "BINARY",
                    "dcInit": {
                        "ordered": True,
                        "maxPacketLifeTime": 0,
                        "maxRetransmits": 0,
                        "protocol": "H264",
                        "negotiated": True,
                        "id": 0,
                        "priority": "NONE",
                    },
                }
            )

        with self.assertRaises(MyException):
            _inner_option_err = ConnectInnerOption(
                {
                    "metadata": "meta message",
                    "serialization": 0,
                    "dcInit": {
                        "ordered": True,
                        "maxPacketLifeTime": 0,
                        "maxRetransmits": 0,
                        "protocol": "H264",
                        "negotiated": True,
                        "id": 0,
                        "priority": "NONE",
                    },
                }
            )

    def test_connect_parameters(self):
        param = ConnectParameters(
            PeerInfo(u"peer_id", u"pt-9749250e-d157-4f80-9ee2-359ce8524308"),
            "target_id",
            DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",),
            Socket(10000, ip_v4=u"127.0.0.1"),
            options={
                "metadata": "string",
                "serialization": "string",
                "dcInit": {
                    "ordered": True,
                    "maxPacketLifeTime": 0,
                    "maxRetransmits": 0,
                    "protocol": "string",
                    "negotiated": True,
                    "id": 0,
                    "priority": "string",
                },
            },
        )
        json = param.json()
        self.assertEqual(
            json["peer_id"], u"peer_id",
        )
        self.assertEqual(
            json["token"], u"pt-9749250e-d157-4f80-9ee2-359ce8524308",
        )
        self.assertEqual(
            json["options"],
            {
                "metadata": "string",
                "serialization": "string",
                "dcInit": {
                    "ordered": True,
                    "maxPacketLifeTime": 0,
                    "maxRetransmits": 0,
                    "protocol": "string",
                    "negotiated": True,
                    "id": 0,
                    "priority": "string",
                },
            },
        )
        self.assertEqual(json["target_id"], "target_id")
        self.assertEqual(
            json["params"], {"data_id": u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211"},
        )
        self.assertEqual(
            json["redirect_params"], {"ip_v4": u"127.0.0.1", "port": 10000,},
        )

    def test_redirect_params(self):
        param = RedirectParameters(
            DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",),
            Socket(10000, ip_v4=u"127.0.0.1"),
        )
        self.assertEqual(
            param.json(),
            {
                "feed_params": {"data_id": u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211"},
                "redirect_params": {"ip_v4": u"127.0.0.1", "port": 10000,},
            },
        )


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "data_model", TestDataModel)
