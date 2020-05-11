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
from domain.peer.model import CreateRequestParams, PeerInfo
from usecase.peer.create_request import CreateRequest
from helper.injector import BindingSpec

PKG = "skyway"


class TestCreateRequest(unittest.TestCase):
    def test_create_request_params_success(self):
        inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
        create_request = inject.provide(CreateRequest)
        json = {"key": "key", "domain": "localhost", "peer_id": "my_id", "turn": True}
        with patch(
            "infra.peer.api.PeerApi.create_request",
            return_value=PeerInfo("my_id", "token"),
        ):
            peer_info = create_request.create_request(json)
            self.assertEqual(peer_info.id(), "my_id")
            self.assertEqual(peer_info.token(), "token")

    def test_create_request_params_param_error(self):
        inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
        create_request = inject.provide(CreateRequest)
        # no domain
        json = {"key": "key", "peer_id": "my_id", "turn": True}
        with patch(
            "infra.peer.api.PeerApi.create_request",
            return_value=PeerInfo("my_id", "token"),
        ):
            with self.assertRaises(MyException):
                _peer_info = create_request.create_request(json)

    def test_create_request_params_api_error(self):
        inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
        create_request = inject.provide(CreateRequest)
        json = {"key": "key", "domain": "localhost", "peer_id": "my_id", "turn": True}
        with patch(
            "infra.peer.api.PeerApi.create_request", side_effect=MyException("error")
        ):
            with self.assertRaises(MyException):
                _peer_info = create_request.create_request(json)


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "data_api", TestCreateRequest)
