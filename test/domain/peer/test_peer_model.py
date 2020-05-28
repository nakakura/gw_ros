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
from domain.peer.model import CreateRequestParams, PeerEvent, PeerInfo

PKG = "skyway"


class TestModel(unittest.TestCase):

    # ----------PeerInfo----------

    def test_create_request_info_blank_id(self):
        with self.assertRaises(MyException):
            peer_info = PeerInfo(u"", u"token")

    def test_create_request_blank_token(self):
        with self.assertRaises(MyException):
            peer_info = PeerInfo(u"my_id", u"")


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "data_api", TestModel)
