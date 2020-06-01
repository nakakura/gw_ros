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
from usecase.data.open_data_socket_request import OpenDataSocketRequest
from domain.data.model import DataSocket
from error import MyException


class TestOpenSocketRequest:
    def setup_method(self, method):
        inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
        self.request = inject.provide(OpenDataSocketRequest)

    def teardown_method(self, method):
        del self.request

    def test_create_request_params_succes(self, mocker):
        data_sock = DataSocket(
            u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211", 10000, ip_v4=u"127.0.0.1",
        )

        mock = mocker.patch("infra.data.api.DataApi.open_data_socket_request")
        mock.return_value = data_sock
        _data_sock = self.request.open_data_socket_request()
        assert mock.called
