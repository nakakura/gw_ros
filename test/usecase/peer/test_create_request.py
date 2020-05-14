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
        json = {
            "key": u"key",
            "domain": u"localhost",
            "peer_id": u"my_id",
            "turn": True,
        }
        with patch(
            "infra.peer.api.PeerApi.create_request",
            return_value=PeerInfo(u"my_id", u"pt-102127d9-30de-413b-93f7-41a33e39d82b"),
        ):
            peer_info = create_request.create_request(json)
            self.assertEqual(peer_info.id(), u"my_id")
            self.assertEqual(
                peer_info.token(), u"pt-102127d9-30de-413b-93f7-41a33e39d82b"
            )

    def test_create_request_params_param_error(self):
        inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
        create_request = inject.provide(CreateRequest)
        # no domain
        json = {"key": u"key", "peer_id": u"my_id", "turn": True}
        with patch(
            "infra.peer.api.PeerApi.create_request",
            return_value=PeerInfo(u"my_id", u"pt-102127d9-30de-413b-93f7-41a33e39d82b"),
        ):
            with self.assertRaises(MyException):
                _peer_info = create_request.create_request(json)

    def test_create_request_params_api_error(self):
        inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
        create_request = inject.provide(CreateRequest)
        json = {
            "key": u"key",
            "domain": u"localhost",
            "peer_id": u"my_id",
            "turn": True,
        }
        with patch(
            "infra.peer.api.PeerApi.create_request", side_effect=MyException("error")
        ):
            with self.assertRaises(MyException):
                _peer_info = create_request.create_request(json)


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "data_api", TestCreateRequest)
