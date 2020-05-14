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
from domain.peer.model import PeerInfo
from usecase.peer.delete_request import DeleteRequest
from helper.injector import BindingSpec

PKG = "skyway"


class TestDeleteRequest(unittest.TestCase):
    def test_delete_request_params_success(self):
        inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
        delete_request = inject.provide(DeleteRequest)
        param = PeerInfo(u"my_id", u"pt-102127d9-30de-413b-93f7-41a33e39d82b")

        with patch(
            "infra.peer.api.PeerApi.delete_request",
            return_value=PeerInfo(u"my_id", u"pt-102127d9-30de-413b-93f7-41a33e39d82b"),
        ) as mock:
            delete_request.delete_request(param)
            self.assertTrue(mock.called)
            self.assertEqual(mock.call_args[0][0], param)


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "data_api", TestDeleteRequest)
