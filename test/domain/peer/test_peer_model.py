#!/usr/bin/env python
# -*- coding: utf-8 -*-
import logging
import sys
import unittest
from os import path
import rospy

sys.path.append(
    path.dirname(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
    + "/scripts"
)
from error import MyException
from domain.peer.model import CreateRequestParams, PeerInfo

PKG = "skyway"


class TestModel(unittest.TestCase):
    def test_create_request_params_success(self):
        json = {"key": "key", "domain": "localhost", "peer_id": "my_id", "turn": True}
        param = CreateRequestParams(json)
        self.assertEqual(json, param.json())

    def test_create_request_params_no_key(self):
        json = {"domain": "localhost", "peer_id": "my_id", "turn": True}
        with self.assertRaises(MyException):
            _param = CreateRequestParams(json)

    def test_create_request_params_blank_key(self):
        json = {"key": "", "domain": "localhost", "peer_id": "my_id", "turn": True}
        with self.assertRaises(MyException):
            _param = CreateRequestParams(json)

    def test_create_request_params_no_domain(self):
        json = {"key": "key", "peer_id": "my_id", "turn": True}
        with self.assertRaises(MyException):
            _param = CreateRequestParams(json)

    def test_create_request_params_blank_domain(self):
        json = {"key": "key", "domain": "", "peer_id": "my_id", "turn": True}
        with self.assertRaises(MyException):
            _param = CreateRequestParams(json)

    def test_create_request_params_no_peer_id(self):
        json = {"key": "key", "domain": "localhost", "turn": True}
        with self.assertRaises(MyException):
            _param = CreateRequestParams(json)

    def test_create_request_params_blank_peer_id(self):
        json = {"key": "key", "domain": "localhost", "peer_id": "", "turn": True}
        with self.assertRaises(MyException):
            _param = CreateRequestParams(json)

    def test_create_request_info_success(self):
        peer_info = PeerInfo("my_id", "token")
        self.assertEqual(peer_info.id(), "my_id")
        self.assertEqual(peer_info.token(), "token")

    def test_create_request_info_blank_id(self):
        with self.assertRaises(MyException):
            peer_info = PeerInfo("", "token")

    def test_create_request_blank_token(self):
        with self.assertRaises(MyException):
            peer_info = PeerInfo("my_id", "")


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "data_api", TestModel)
