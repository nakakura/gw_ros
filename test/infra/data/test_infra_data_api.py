#!/usr/bin/env python
# -*- coding: utf-8 -*-
import logging
import sys
import unittest
from os import path
from mock import *

sys.path.append(
    path.dirname(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
)
sys.path.append(
    path.dirname(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
    + "/scripts"
)
from infra.data.api import DataApi
from domain.data.model import DataId, DataConnectionId, ConnectParameters, Socket
from domain.common.model import PeerInfo

PKG = "skyway"


class TestDataApi(unittest.TestCase):
    def test_open_data_socket(self):
        data_api = DataApi("dummy")
        with patch(
            "infra.rest.Rest.post",
            return_value={
                u"data_id": u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",
                u"port": 10001,
                u"ip_v4": u"127.0.0.1",
            },
        ) as mock:
            value = data_api.open_data_socket_request()
            data_id = value.data_id()
            socket = value.socket()
            self.assertTrue(mock.called)
            self.assertEqual(mock.call_args[0][0], "data")
            self.assertEqual(data_id.id(), u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211")
            self.assertEqual(socket.port(), 10001)
            self.assertEqual(socket.ip_v4(), u"127.0.0.1")
            self.assertEqual(socket.ip_v6(), u"")

    def test_close_data_socket(self):
        data_api = DataApi("dummy")
        data_id = DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211")
        with patch("infra.rest.Rest.delete", return_value={},) as mock:
            data_api.close_data_socket_request(data_id)
            self.assertTrue(mock.called)
            self.assertEqual(
                mock.call_args[0][0], "data/da-50a32bab-b3d9-4913-8e20-f79c90a6a211"
            )

    def test_connect(self):
        data_api = DataApi("dummy")
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

        with patch(
            "infra.rest.Rest.post",
            return_value={
                u"command_type": u"PEERS_CONNECT",
                u"params": {
                    u"data_connection_id": u"dc-4995f372-fb6a-4196-b30a-ce11e5c7f56c"
                },
            },
        ) as mock:
            data_connection_id = data_api.connect_request(param)
            self.assertTrue(mock.called)
            self.assertEqual(mock.call_args[0][0], "data/connections")
            self.assertEqual(mock.call_args[0][1], param.json())
            self.assertEqual(mock.call_args[0][2], 202)
            self.assertEqual(
                data_connection_id.id(), u"dc-4995f372-fb6a-4196-b30a-ce11e5c7f56c"
            )

    def test_close_data_connection(self):
        data_api = DataApi("dummy")
        data_connection_id = DataConnectionId(
            u"dc-50a32bab-b3d9-4913-8e20-f79c90a6a211"
        )
        with patch("infra.rest.Rest.delete", return_value={},) as mock:
            data_api.disconnect_request(data_connection_id)
            self.assertTrue(mock.called)
            self.assertEqual(
                mock.call_args[0][0],
                "data/connections/dc-50a32bab-b3d9-4913-8e20-f79c90a6a211",
            )
            self.assertEqual(mock.call_args[0][1], 204)


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "test_peer_api", TestDataApi)
