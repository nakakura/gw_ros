#!/usr/bin/env python
# -*- coding: utf-8 -*-
import logging
import sys
import unittest
from os import path
import rospy
import pinject
from mock import patch, MagicMock

sys.path.append(
    path.dirname(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
)
sys.path.append(
    path.dirname(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
    + "/scripts"
)
from error import MyException
from usecase.data.open_data_socket_request import OpenDataSocketRequest
from domain.data.model import DataSocket
from helper.injector import BindingSpec

PKG = "skyway"


class TestCreateRequest(unittest.TestCase):
    def test_create_request_params_success(self):
        inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
        open_data_request = inject.provide(OpenDataSocketRequest)
        data_sock = DataSocket(
            u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211", 10000, ip_v4=u"127.0.0.1",
        )
        with patch(
            "infra.data.api.DataApi.open_data_socket_request", return_value=data_sock
        ) as mock:
            data_socket = open_data_request.open_data_socket_request()
            self.assertTrue(mock.called)
            self.assertEqual(data_socket, data_sock)


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "data_api", TestCreateRequest)
