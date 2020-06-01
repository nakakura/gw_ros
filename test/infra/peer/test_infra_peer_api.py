# -*- coding: utf-8 -*-
import sys
from os import path

sys.path.insert(
    0,
    path.dirname(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
    + "/scripts",
)
from domain.peer.model import CreateRequestParams, PeerStatus
from domain.common.model import PeerInfo
from infra.peer.api import PeerApi


class TestPeerApi:
    def setup_method(self, method):
        self.peer_api = PeerApi("dummy")
        self.json = {
            "key": "key",
            "domain": "localhost",
            "peer_id": "my_id",
            "turn": True,
        }
        self.create_peer_params = CreateRequestParams(self.json)
        self.peer_info = PeerInfo(u"my_id", u"pt-102127d9-30de-413b-93f7-41a33e39d82b")
        self.return_value = {
            "command_type": "PEERS_CREATE",
            "params": self.peer_info.json(),
        }

    def teardown_method(self, method):
        del self.peer_api
        del self.json
        del self.create_peer_params
        del self.peer_info
        del self.return_value

    def test_create_request_success(self, mocker):
        mock = mocker.patch("infra.rest.Rest.post")
        mock.return_value = self.return_value
        value = self.peer_api.create_request(self.create_peer_params)
        assert value == self.peer_info

    def test_delete_request_status(self, mocker):
        mock = mocker.patch("infra.rest.Rest.delete")
        mock.return_value = {}
        self.peer_api.delete_request(self.peer_info)
        assert mock.called
        assert mock.call_args_list == [
            mocker.call(
                "peers/my_id?token=pt-102127d9-30de-413b-93f7-41a33e39d82b", 204
            )
        ]

    def test_status_request_status(self, mocker):
        mock = mocker.patch("infra.rest.Rest.get")
        mock.return_value = {u"peer_id": u"my_id", u"disconnected": False}
        status = self.peer_api.status_request(self.peer_info)
        assert mock.called
        assert mock.call_args_list == [
            mocker.call(
                "peers/my_id/status?token=pt-102127d9-30de-413b-93f7-41a33e39d82b", 204
            )
        ]
        assert status == PeerStatus(u"my_id", False)
