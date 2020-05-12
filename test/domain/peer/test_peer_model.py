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

    # ----------CreateRequestParams----------
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

    # ----------PeerInfo----------
    def test_create_request_info_success(self):
        peer_info = PeerInfo("my_id", "pt-102127d9-30de-413b-93f7-41a33e39d82b")
        self.assertEqual(peer_info.id(), "my_id")
        self.assertEqual(peer_info.token(), "pt-102127d9-30de-413b-93f7-41a33e39d82b")

    def test_create_request_info_blank_id(self):
        with self.assertRaises(MyException):
            peer_info = PeerInfo("", "token")

    def test_create_request_blank_token(self):
        with self.assertRaises(MyException):
            peer_info = PeerInfo("my_id", "")

    # ----------PeerEvent----------
    def test_peer_event_open_valid(self):
        peer_event = PeerEvent(
            {
                "event": "OPEN",
                "params": {
                    "peer_id": "hoge",
                    "token": "pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
                },
            }
        )
        self.assertEqual(
            peer_event.peer_info(),
            PeerInfo("hoge", "pt-870c2c49-c16d-4c69-b1ad-fec7550564af"),
        )
        with self.assertRaises(AttributeError):
            peer_event.media_connection_id()
        with self.assertRaises(AttributeError):
            peer_event.data_connection_id()
        with self.assertRaises(AttributeError):
            peer_event.error_message()

    def test_peer_event_open_no_peer_id(self):
        with self.assertRaises(KeyError):
            _peer_event = PeerEvent(
                {
                    "event": "OPEN",
                    "params": {"token": "pt-870c2c49-c16d-4c69-b1ad-fec7550564af",},
                }
            )

    def test_peer_event_open_no_token(self):
        with self.assertRaises(KeyError):
            _peer_event = PeerEvent({"event": "OPEN", "params": {"peer_id": "hoge",},})

    def test_peer_event_close_valid(self):
        peer_event = PeerEvent(
            {
                "event": "CLOSE",
                "params": {
                    "peer_id": "hoge",
                    "token": "pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
                },
            }
        )
        self.assertEqual(
            peer_event.peer_info(),
            PeerInfo("hoge", "pt-870c2c49-c16d-4c69-b1ad-fec7550564af"),
        )
        with self.assertRaises(AttributeError):
            peer_event.media_connection_id()
        with self.assertRaises(AttributeError):
            peer_event.data_connection_id()
        with self.assertRaises(AttributeError):
            peer_event.error_message()

    def test_peer_event_CLOSE_no_peer_id(self):
        with self.assertRaises(KeyError):
            _peer_event = PeerEvent(
                {
                    "event": "CLOSE",
                    "params": {"token": "pt-870c2c49-c16d-4c69-b1ad-fec7550564af",},
                }
            )

    def test_peer_event_close_no_token(self):
        with self.assertRaises(KeyError):
            _peer_event = PeerEvent({"event": "CLOSE", "params": {"peer_id": "hoge",},})

    def test_peer_event_connect_valid(self):
        peer_event = PeerEvent(
            {
                "event": "CONNECTION",
                "params": {
                    "peer_id": "hoge",
                    "token": "pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
                },
                "data_params": {
                    "data_connection_id": "dc-102127d9-30de-413b-93f7-41a33e39d82b"
                },
            }
        )
        self.assertEqual(
            peer_event.peer_info(),
            PeerInfo("hoge", "pt-870c2c49-c16d-4c69-b1ad-fec7550564af"),
        )
        with self.assertRaises(AttributeError):
            peer_event.media_connection_id()
        self.assertEqual(
            peer_event.data_connection_id(), "dc-102127d9-30de-413b-93f7-41a33e39d82b"
        )
        with self.assertRaises(AttributeError):
            peer_event.error_message()

    def test_peer_event_connect_no_peer_id(self):
        with self.assertRaises(KeyError):
            _peer_event = PeerEvent(
                {
                    "event": "CONNECTION",
                    "params": {"token": "pt-870c2c49-c16d-4c69-b1ad-fec7550564af",},
                    "data_params": {
                        "data_connection_id": "dc-102127d9-30de-413b-93f7-41a33e39d82b"
                    },
                }
            )

    def test_peer_event_connect_invalid_data_connection_id(self):
        with self.assertRaises(MyException):
            _peer_event = PeerEvent(
                {
                    "event": "CONNECTION",
                    "params": {
                        "peer_id": "hoge",
                        "token": "pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
                    },
                    "data_params": {
                        "data_connection_id": "da-102127d9-30de-413b-93f7-41a33e39d82b"
                    },
                }
            )

    def test_peer_event_connect_short_data_connection_id(self):
        with self.assertRaises(MyException):
            _peer_event = PeerEvent(
                {
                    "event": "CONNECTION",
                    "params": {
                        "peer_id": "hoge",
                        "token": "pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
                    },
                    "data_params": {
                        "data_connection_id": "dc-102127d9-30de-413b-93f7-41a33e3"
                    },
                }
            )

    def test_peer_event_call_valid(self):
        peer_event = PeerEvent(
            {
                "event": "CALL",
                "params": {
                    "peer_id": "hoge",
                    "token": "pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
                },
                "call_params": {
                    "media_connection_id": "mc-102127d9-30de-413b-93f7-41a33e39d82b"
                },
            }
        )
        self.assertEqual(
            peer_event.peer_info(),
            PeerInfo("hoge", "pt-870c2c49-c16d-4c69-b1ad-fec7550564af"),
        )
        with self.assertRaises(AttributeError):
            peer_event.data_connection_id()
        self.assertEqual(
            peer_event.media_connection_id(), "mc-102127d9-30de-413b-93f7-41a33e39d82b"
        )
        with self.assertRaises(AttributeError):
            peer_event.error_message()

    def test_peer_event_call_no_peer_id(self):
        with self.assertRaises(KeyError):
            _peer_event = PeerEvent(
                {
                    "event": "CALL",
                    "params": {"token": "pt-870c2c49-c16d-4c69-b1ad-fec7550564af",},
                    "call_params": {
                        "media_connection_id": "mc-102127d9-30de-413b-93f7-41a33e39d82b"
                    },
                }
            )

    def test_peer_event_call_invalid_data_connection_id(self):
        with self.assertRaises(MyException):
            _peer_event = PeerEvent(
                {
                    "event": "CALL",
                    "params": {
                        "peer_id": "hoge",
                        "token": "pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
                    },
                    "call_params": {
                        "media_connection_id": "ma-102127d9-30de-413b-93f7-41a33e3"
                    },
                }
            )

    def test_peer_event_call_short_data_connection_id(self):
        with self.assertRaises(MyException):
            _peer_event = PeerEvent(
                {
                    "event": "CALL",
                    "params": {
                        "peer_id": "hoge",
                        "token": "pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
                    },
                    "call_params": {
                        "media_connection_id": "mc-102127d9-30de-413b-93f7-41a33e3"
                    },
                }
            )

    def test_peer_event_error_valid(self):
        peer_event = PeerEvent(
            {
                "event": "ERROR",
                "params": {
                    "peer_id": "hoge",
                    "token": "pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
                },
                "error_message": "BROWSER_INCOMPATIBLE",
            }
        )
        self.assertEqual(
            peer_event.peer_info(),
            PeerInfo("hoge", "pt-870c2c49-c16d-4c69-b1ad-fec7550564af"),
        )
        with self.assertRaises(AttributeError):
            peer_event.data_connection_id()
        with self.assertRaises(AttributeError):
            peer_event.media_connection_id()
        self.assertEqual(peer_event.error_message(), "BROWSER_INCOMPATIBLE")

    def test_peer_event_error_no_peer_id(self):
        with self.assertRaises(KeyError):
            _peer_event = PeerEvent(
                {
                    "event": "ERROR",
                    "params": {"token": "pt-870c2c49-c16d-4c69-b1ad-fec7550564af",},
                    "error_message": "BROWSER_INCOMPATIBLE",
                }
            )

    def test_peer_event_error_no_token(self):
        with self.assertRaises(KeyError):
            _peer_event = PeerEvent(
                {
                    "event": "ERROR",
                    "params": {"peer_id": "my_id",},
                    "error_message": "BROWSER_INCOMPATIBLE",
                }
            )

    def test_peer_event_error_no_error_message(self):
        with self.assertRaises(KeyError):
            _peer_event = PeerEvent(
                {
                    "event": "ERROR",
                    "params": {
                        "peer_id": "my_id",
                        "token": "pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
                    },
                }
            )


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "data_api", TestModel)
