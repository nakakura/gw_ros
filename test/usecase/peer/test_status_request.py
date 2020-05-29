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
from usecase.peer.status_request import StatusRequest
from domain.common.model import PeerInfo
from domain.peer.model import PeerStatus
from error import MyException


class TestStatusRequest:
    def setup_method(self, method):
        inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
        self.status_request = inject.provide(StatusRequest)
        self.peer_info = (
            PeerInfo(u"my_id", u"pt-102127d9-30de-413b-93f7-41a33e39d82b"),
        )
        self.status = PeerStatus(u"my_id", False)

    def teardown_method(self, method):
        del self.status_request
        del self.peer_info
        del self.status

    def test_status_request_params_succes(self, mocker):
        mocker.patch("infra.peer.api.PeerApi.status_request").return_value = self.status
        result = self.status_request.run(self.peer_info)
        assert result == self.status

    def test_status_request_params_error(self, mocker):
        mocker.patch("infra.peer.api.PeerApi.status_request").side_effect = MyException(
            "error"
        )
        with pytest.raises(MyException):
            _peer_info = self.status_request.run(self.peer_info)
