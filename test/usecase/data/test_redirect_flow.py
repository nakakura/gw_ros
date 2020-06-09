# -*- coding: utf-8 -*-
import pytest
import sys
import pinject
from os import path

sys.path.insert(
    0,
    path.dirname(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
    + "/scripts",
)
from helper.injector import BindingSpec
from helper.methods import *
from usecase.data.redirect_flow import RedirectFlow
from domain.data.model import DataSocket, Status
from domain.common.model import DataConnectionId, DataId
from error import MyException


class TestRedirectFlow:
    def setup_method(self, method):
        self.data_connection_id = DataConnectionId(
            u"dc-50a32bab-b3d9-4913-8e20-f79c90a6a211"
        )
        self.status = Status(
            {
                u"remote_id": u"peer_id",
                u"buffersize": 0,
                u"label": u"",
                u"metadata": u"data",
                u"open": True,
                u"reliable": True,
                u"serialization": u"BINARY_UTF8",
                u"type": u"DATA",
            }
        )
        self.config = [
            {"name": "data", "redirect_params": {"ip_v4": "127.0.0.1", "port": 10000}},
            {"name": "data2", "redirect_params": {"ip_v6": "fe00::1", "port": 10001}},
        ]
        self.redirect_flow = RedirectFlow()

    def teardown_method(self, method):
        del self.data_connection_id
        del self.redirect_flow
        del self.status
        del self.config

    def test_redirect_flow_success(self, mocker):
        # check metadata
        mock_status = mocker.patch("infra.data.api.DataApi.status_request")
        mock_status.return_value = {
            u"remote_id": u"peer_id",
            u"buffersize": 0,
            u"label": u"",
            u"metadata": u"data",
            u"open": True,
            u"reliable": True,
            u"serialization": u"BINARY_UTF8",
            u"type": u"DATA",
        }

        #  open data sock
        mock_open_sock = mocker.patch("infra.data.api.DataApi.open_data_socket_request")
        mock_open_sock.return_value = {
            u"data_id": u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",
            u"port": 10001,
            u"ip_v4": "127.0.0.1",
        }

        # redirect
        mock_redirect = mocker.patch("infra.data.api.DataApi.redirect_request")
        mock_redirect.return_value = DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211")
        assert self.redirect_flow.run(self.config, self.data_connection_id) == {
            "flag": True,
            "data_id": DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211"),
            "status": self.status,
            "item": {
                "name": "data",
                "redirect_params": {"ip_v4": "127.0.0.1", "port": 10000},
            },
            "config": [
                {
                    "name": "data2",
                    "redirect_params": {"ip_v6": "fe00::1", "port": 10001},
                },
            ],
        }

    def test_redirect_flow_no_config(self, mocker):
        # check metadata
        mock_status = mocker.patch("infra.data.api.DataApi.status_request")
        mock_status.return_value = {
            u"remote_id": u"peer_id",
            u"buffersize": 0,
            u"label": u"",
            u"metadata": u"data3",
            u"open": True,
            u"reliable": True,
            u"serialization": u"BINARY_UTF8",
            u"type": u"DATA",
        }

        #  open data sock
        mock_open_sock = mocker.patch("infra.data.api.DataApi.open_data_socket_request")
        mock_open_sock.return_value = {
            u"data_id": u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",
            u"port": 10001,
            u"ip_v4": "127.0.0.1",
        }

        mock_disconnect = mocker.patch("infra.data.api.DataApi.disconnect_request")
        mock_disconnect.return_value = {}

        # redirect
        mock_redirect = mocker.patch("infra.data.api.DataApi.redirect_request")
        mock_redirect.return_value = {
            u"command_type": u"DATA_CONNECTION_PUT",
            u"data_id": u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",
        }
        assert self.redirect_flow.run(self.config, self.data_connection_id) == {
            "flag": False,
            "item": {},
            "config": self.config,
        }
        assert mock_disconnect.called
        assert mock_disconnect.call_args_list == [mocker.call(self.data_connection_id)]
