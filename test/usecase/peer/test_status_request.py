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
from domain.peer.model import PeerInfo, PeerStatus
from usecase.peer.status_request import StatusRequest
from helper.injector import BindingSpec

PKG = "skyway"


class TestStatusRequest(unittest.TestCase):
    def test_status_request_params_success(self):
        inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
        status_request = inject.provide(StatusRequest)
        param = PeerInfo("my_id", "pt-102127d9-30de-413b-93f7-41a33e39d82b")

        with patch(
            "infra.peer.api.PeerApi.status_request",
            return_value=PeerStatus("my_id", False),
        ) as mock:
            result = status_request.status_request(param)
            self.assertTrue(mock.called)
            self.assertEqual(mock.call_args[0][0], param)
            self.assertEqual(result, PeerStatus("my_id", False))


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "data_api", TestStatusRequest)
