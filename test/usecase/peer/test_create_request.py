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
from usecase.peer.create_request import CreateRequest
from domain.peer.model import PeerInfo
from error import MyException


class TestCreateRequest:
    def setup_method(self, method):
        inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
        self.create_request = inject.provide(CreateRequest)
        self.peer_info = (
            PeerInfo(u"my_id", u"pt-102127d9-30de-413b-93f7-41a33e39d82b"),
        )
        self.json = {
            "key": "key",
            "domain": "localhost",
            "peer_id": "my_id",
            "turn": True,
        }

    def teardown_method(self, method):
        del self.create_request
        del self.peer_info
        del self.json

    def test_create_request_params_succes(self, mocker):
        mocker.patch(
            "infra.peer.api.PeerApi.create_request"
        ).return_value = self.peer_info
        peer_info = self.create_request.run(self.json)
        assert peer_info == self.peer_info

    def test_create_request_params_error(self, mocker):
        mocker.patch("infra.peer.api.PeerApi.create_request").side_effect = MyException(
            "error"
        )
        with pytest.raises(MyException):
            _peer_info = self.create_request.run(self.json)
