# -*- coding: utf-8 -*-
import pytest
import logging
import sys
import unittest
from os import path
import rospy
import json as encoder

sys.path.insert(
    0,
    path.dirname(
        path.dirname(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
    )
    + "/scripts",
)
from error import MyException
from domain.common.model import PeerInfo
from domain.peer.model import CreateRequestParams, PeerEvent


class TestPeerModel:
    def setup_method(self, method):
        self.json = {
            "key": "key",
            "domain": "localhost",
            "peer_id": "my_id",
            "turn": True,
        }

    def teardown_method(self, method):
        del self.json

    @pytest.fixture(
        params=[
            "no_key",
            "blank_key",
            "no_domain",
            "blank_domain",
            "no_peer_id",
            "blank_peer_id",
            "no_turn",
        ]
    )
    def context(self, request):
        if request.param == "no_key":
            del self.json["key"]
            return self.json
        elif request.param == "blank_key":
            self.json["key"] = ""
            return self.json
        elif request.param == "no_domain":
            del self.json["domain"]
            return self.json
        elif request.param == "blank_domain":
            self.json["domain"] = ""
            return self.json
        elif request.param == "no_peer_id":
            del self.json["peer_id"]
            return self.json
        elif request.param == "blank_peer_id":
            self.json["peer_id"] = ""
            return self.json
        else:
            del self.json["turn"]
            return self.json

    def test_create_request_params_success(self):
        param = CreateRequestParams(self.json)
        assert param.json() == self.json, "CreateRequestParams return unexpected json"

    def test_create_request_params_fail(self, context):
        with pytest.raises(MyException):
            _param = CreateRequestParams(self.json)


class TestPeerEvent:
    def setup_method(self, method):
        self.peer_info = PeerInfo(u"hoge", u"pt-870c2c49-c16d-4c69-b1ad-fec7550564af")
        self.open_json = {"event": "OPEN", "params": self.peer_info.json()}
        self.close_json = {"event": "CLOSE", "params": self.peer_info.json()}
        self.data_connection_id = u"dc-102127d9-30de-413b-93f7-41a33e39d82b"
        self.connect_json = {
            "event": "CONNECTION",
            "params": self.peer_info.json(),
            "data_params": {"data_connection_id": self.data_connection_id},
        }
        self.media_connection_id = u"mc-102127d9-30de-413b-93f7-41a33e39d82b"
        self.call_json = {
            "event": "CALL",
            "params": self.peer_info.json(),
            "call_params": {"media_connection_id": self.media_connection_id},
        }
        self.error_message = u"BROWSER_INCOMPATIBLE"
        self.error_json = {
            "event": "ERROR",
            "params": self.peer_info.json(),
            "error_message": self.error_message,
        }

    def teardown_method(self, method):
        del self.open_json
        del self.close_json
        del self.data_connection_id
        del self.connect_json
        del self.media_connection_id
        del self.call_json
        del self.error_message
        del self.error_json

    @pytest.fixture(
        params=[
            "no_event",
            "no_peer_id",
            "no_token",
            "no_data_params",
            "no_call_params",
            "no_error_messages",
        ]
    )
    def context(self, request):
        if request.param == "no_event":
            del self.open_json["event"]
            return self.open_json
        elif request.param == "no_peer_id":
            del self.open_json["params"]["peer_id"]
            return self.open_json
        elif request.param == "no_token":
            del self.open_json["params"]["token"]
            return self.open_json
        elif request.param == "no_data_params":
            del self.connect_json["data_params"]
            return self.connect_json
        elif request.param == "no_call_params":
            del self.call_json["call_params"]
            return self.call_json
        elif request.param == "no_error_messages":
            del self.error_json["error_message"]
            return self.error_json

    def test_peer_event_open_valid(self):
        peer_event = PeerEvent(self.open_json)
        assert peer_event.peer_info() == self.peer_info, "invalid peer_info"
        with pytest.raises(AttributeError):
            peer_event.media_connection_id()
        with pytest.raises(AttributeError):
            peer_event.data_connection_id()
        with pytest.raises(AttributeError):
            peer_event.error_message()

    def test_peer_event_close_valid(self):
        peer_event = PeerEvent(self.close_json)
        assert peer_event.peer_info() == self.peer_info, "invalid peer_info"
        with pytest.raises(AttributeError):
            peer_event.media_connection_id()
        with pytest.raises(AttributeError):
            peer_event.data_connection_id()
        with pytest.raises(AttributeError):
            peer_event.error_message()

    def test_peer_event_connect_valid(self):
        peer_event = PeerEvent(self.connect_json)
        assert peer_event.peer_info() == self.peer_info, "invalid peer_info"
        assert (
            peer_event.data_connection_id() == self.data_connection_id
        ), "invalid data_connection_id"
        with pytest.raises(AttributeError):
            peer_event.media_connection_id()
        with pytest.raises(AttributeError):
            peer_event.error_message()

    def test_peer_event_call_valid(self):
        peer_event = PeerEvent(self.call_json)
        assert peer_event.peer_info() == self.peer_info, "invalid peer_info"
        assert (
            peer_event.media_connection_id() == self.media_connection_id
        ), "invalid media_connection_id"
        with pytest.raises(AttributeError):
            peer_event.data_connection_id()
        with pytest.raises(AttributeError):
            peer_event.error_message()

    def test_peer_event_error_valid(self):
        peer_event = PeerEvent(self.error_json)
        assert peer_event.peer_info() == self.peer_info, "invalid peer_info"
        assert peer_event.error_message() == self.error_message, "invalid error_message"
        with pytest.raises(AttributeError):
            peer_event.data_connection_id()
        with pytest.raises(AttributeError):
            peer_event.media_connection_id()

    def test_peer_event_invalid_params(self, context):
        with pytest.raises(KeyError):
            _peer_event = PeerEvent(context)
