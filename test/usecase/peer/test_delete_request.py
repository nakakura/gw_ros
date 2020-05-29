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
from usecase.peer.delete_request import DeleteRequest
from domain.peer.model import PeerInfo
from error import MyException


class TestDeleteRequest:
    def setup_method(self, method):
        inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
        self.delete_request = inject.provide(DeleteRequest)
        self.peer_info = (
            PeerInfo(u"my_id", u"pt-102127d9-30de-413b-93f7-41a33e39d82b"),
        )

    def teardown_method(self, method):
        del self.delete_request
        del self.peer_info

    def test_delete_request_params_succes(self, mocker):
        mocker.patch(
            "infra.peer.api.PeerApi.delete_request"
        ).return_value = self.peer_info
        peer_info = self.delete_request.delete_request(self.peer_info)
        assert peer_info == self.peer_info

    def test_delete_request_params_error(self, mocker):
        mocker.patch("infra.peer.api.PeerApi.delete_request").side_effect = MyException(
            "error"
        )
        with pytest.raises(MyException):
            _peer_info = self.delete_request.delete_request(self.peer_info)
