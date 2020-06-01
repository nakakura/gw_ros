# -*- coding: utf-8 -*-
import sys
from os import path

sys.path.insert(
    0,
    path.dirname(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
    + "/scripts",
)
from domain.data.model import DataSocket, ConnectParameters, Socket, RedirectParameters
from domain.common.model import DataId, PeerInfo, DataConnectionId
from infra.data.api import DataApi


class TestDataApi:
    def setup_method(self, method):
        self.data_api = DataApi("dummy")
        self.data_id = DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211")
        self.data_connection_id = DataConnectionId(
            u"dc-4995f372-fb6a-4196-b30a-ce11e5c7f56c"
        )

    def teardown_method(self, method):
        del self.data_api
        del self.data_id
        del self.data_connection_id

    def test_create_request_success(self, mocker):
        json = {
            u"data_id": u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",
            u"port": 10001,
            u"ip_v4": u"127.0.0.1",
        }
        mock = mocker.patch("infra.rest.Rest.post")
        mock.return_value = {
            u"data_id": u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",
            u"port": 10001,
            u"ip_v4": u"127.0.0.1",
        }
        value = self.data_api.open_data_socket_request()
        assert value == DataSocket(
            json[u"data_id"], json[u"port"], ip_v4=json[u"ip_v4"]
        )

    def test_close_data_socket(self, mocker):
        mock = mocker.patch("infra.rest.Rest.delete")
        mock.return_value = {}
        self.data_api.close_data_socket_request(self.data_id)
        assert mock.called
        assert mock.call_args_list == [
            mocker.call("data/da-50a32bab-b3d9-4913-8e20-f79c90a6a211", 204)
        ]

    def test_connect(self, mocker):
        param = ConnectParameters(
            PeerInfo(u"peer_id", u"pt-9749250e-d157-4f80-9ee2-359ce8524308"),
            "target_id",
            self.data_id,
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
        mock = mocker.patch("infra.rest.Rest.post")
        mock.return_value = {
            u"command_type": u"PEERS_CONNECT",
            u"params": {u"data_connection_id": self.data_connection_id.id()},
        }
        data_connection_id = self.data_api.connect_request(param)
        assert mock.called
        assert mock.call_args_list == [
            mocker.call("data/connections", param.json(), 202)
        ]
        assert data_connection_id == self.data_connection_id

    def test_close_data_connection(self, mocker):
        mock = mocker.patch("infra.rest.Rest.delete")
        mock.return_value = {}
        self.data_api.disconnect_request(self.data_connection_id)
        assert mock.called
        assert mock.call_args_list == [
            mocker.call(
                "data/connections/{}".format(self.data_connection_id.id()), 204,
            )
        ]

    def test_data_redirect(self, mocker):
        socket = Socket(10000, ip_v4=u"127.0.0.1")
        redirect_param = RedirectParameters(self.data_id, socket)
        mock = mocker.patch("infra.rest.Rest.put")
        mock.return_value = {
            "command_type": "DATA_CONNECTION_PUT",
            "data_id": self.data_id.id(),
        }
        ret_id = self.data_api.redirect_request(self.data_connection_id, redirect_param)
        assert ret_id == self.data_id
        assert mock.called
        assert mock.call_args_list == [
            mocker.call(
                "data/connections/{}".format(self.data_connection_id.id()),
                redirect_param.json(),
                200,
            )
        ]
