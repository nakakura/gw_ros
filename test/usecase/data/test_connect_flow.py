# -*- coding: utf-8 -*-
import pytest
import sys
import pinject
from os import path

sys.path.insert(
    0,
    path.dirname(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
    + u"/scripts",
)

from usecase.data.connect_flow import ConnectFlow
from domain.data.model import Status, DataSocket
from domain.common.model import DataConnectionId


class TestConnectFlow:
    def setup_method(self, method):
        self.config = {
            u"peer_id": u"peer_id",
            u"token": u"pt-9749250e-d157-4f80-9ee2-359ce8524308",
            u"options": {
                u"metadata": "string",
                u"serialization": "string",
                u"dcInit": {
                    u"ordered": True,
                    u"maxPacketLifeTime": 0,
                    u"maxRetransmits": 0,
                    u"protocol": "H264",
                    u"negotiated": True,
                    u"id": 0,
                    u"priority": "HIGH",
                },
            },
            u"target_id": u"ID_BAR",
            u"redirect_params": {
                u"ip_v4": u"string",
                u"ip_v6": u"string",
                u"port": 10001,
            },
        }

    def teardown_method(self, method):
        del self.config

    def test_connect_flow(self, mocker):
        connect_flow = ConnectFlow()

        status = Status(
            {
                u"remote_id": u"ID_BAR",
                u"buffersize": 0,
                u"label": u"string",
                u"metadata": u"metadata",
                u"open": True,
                u"reliable": True,
                u"serialization": u"BINARY_UTF8",
                u"type": u"DATA",
            }
        )

        port = 10000
        ip_v4 = u"127.0.0.1"
        data_sock = DataSocket(
            u"da-9749250e-d157-4f80-9ee2-359ce8524308", port, ip_v4=ip_v4
        )
        mock_open_data_sock = mocker.patch(
            "infra.data.api.DataApi.open_data_socket_request"
        )
        mock_open_data_sock.return_value = data_sock

        mock_connect = mocker.patch("infra.data.api.DataApi.connect_request")
        mock_connect.return_value = DataConnectionId(
            u"dc-50a32bab-b3d9-4913-8e20-f79c90a6a211"
        )

        mock_status = mocker.patch("infra.data.api.DataApi.status_request")
        mock_status.return_value = status

        assert connect_flow.run(self.config) == {
            u"flag": True,
            u"socket": {u"ip_v4": ip_v4, u"port": port},
            u"status": status.json(),
        }

    def test_connect_flow_no_peer_id(self, mocker):
        del self.config["peer_id"]
        connect_flow = ConnectFlow()
        # peer_id is a mandatory field for the CONNECT API
        assert connect_flow.run(self.config) == {
            u"flag": False,
            u"error": u"no_peer_id",
        }

    def test_connect_flow_no_token(self, mocker):
        del self.config["token"]
        connect_flow = ConnectFlow()
        # peer_id is a mandatory field for the CONNECT API
        assert connect_flow.run(self.config) == {
            u"flag": False,
            u"error": u"no_token",
        }
