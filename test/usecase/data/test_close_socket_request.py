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
from usecase.data.close_data_socket_request import CloseDataSocketRequest
from domain.common.model import DataId


class TestCloseSocketRequest:
    def setup_method(self, method):
        inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
        self.request = inject.provide(CloseDataSocketRequest)

    def teardown_method(self, method):
        del self.request

    def test_close_socket_request(self, mocker):
        mock = mocker.patch("infra.data.api.DataApi.close_data_socket_request")
        mock.return_value = {}
        data_id = DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211")
        self.request.close_data_socket_request(data_id)
        assert mock.called
        assert mock.call_args_list == [mocker.call(data_id)]
