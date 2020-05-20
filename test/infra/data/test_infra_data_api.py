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


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "test_peer_api", TestDataApi)
