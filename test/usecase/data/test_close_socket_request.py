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
from usecase.data.close_data_socket_request import CloseDataSocketRequest
from domain.data.model import DataSocket, DataId
from helper.injector import BindingSpec

PKG = "skyway"


class TestCloseRequest(unittest.TestCase):
    def test_close_request_params_success(self):
        inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
        close_data_request = inject.provide(CloseDataSocketRequest)
        data_id = DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211")
        with patch(
            "infra.data.api.DataApi.close_data_socket_request", return_value={}
        ) as mock:
            close_data_request.close_data_socket_request(data_id)
            self.assertTrue(mock.called)


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "data_api", TestCloseRequest)
