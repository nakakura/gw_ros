#!/usr/bin/env python
# -*- coding: utf-8 -*-
import logging
import sys
import unittest
from os import path
from mock import *

sys.path.append(
    path.dirname(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
)
sys.path.append(
    path.dirname(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
    + "/scripts"
)
from domain.peer.model import CreateRequestParams
from infra.peer.api import PeerApi

PKG = "skyway"


class TestPeerApi(unittest.TestCase):
    def test_create_request_success(self):
        peer_api = PeerApi("dummy")
        json = {"key": "key", "domain": "localhost", "peer_id": "my_id", "turn": True}
        params = CreateRequestParams(json)
        with patch(
            "infra.rest.Rest.post",
            return_value={
                "command_type": "PEERS_CREATE",
                "params": {
                    "peer_id": "my_id",
                    "token": "pt-102127d9-30de-413b-93f7-41a33e39d82b",
                },
            },
        ) as mock_post:
            value = peer_api.create_request(params)
            self.assertEqual(value.id(), "my_id")
            self.assertEqual(value.token(), "pt-102127d9-30de-413b-93f7-41a33e39d82b")


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "test_peer_api", TestPeerApi)
