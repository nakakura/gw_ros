# -*- coding: utf-8 -*-
import pytest
import sys
import pinject
from os import path
from mock import MagicMock
import concurrent.futures
import multiprocessing
import Queue
import json as encoder

sys.path.insert(
    0,
    path.dirname(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
    + "/scripts",
)
from helper.injector import BindingSpec
from usecase.peer.subscribe_events import SubscribeEvents
from domain.common.model import PeerInfo
from domain.peer.model import PeerEvent
from error import MyException


class TestStatusRequest:
    def setup_method(self, method):
        inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
        self.subscribe_events = inject.provide(SubscribeEvents)
        self.peer_info = PeerInfo(u"my_id", u"pt-102127d9-30de-413b-93f7-41a33e39d82b")
        self.event_sink = multiprocessing.Queue()
        self.open_event = PeerEvent(
            {
                "event": "OPEN",
                "params": {
                    "peer_id": u"hoge",
                    "token": u"pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
                },
            }
        )
        self.connection_event = PeerEvent(
            {
                "event": "CONNECTION",
                "params": {
                    "peer_id": u"hoge",
                    "token": u"pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
                },
                "data_params": {
                    "data_connection_id": u"dc-102127d9-30de-413b-93f7-41a33e39d82b"
                },
            }
        )
        self.call_event = PeerEvent(
            {
                "event": "CALL",
                "params": {
                    "peer_id": u"hoge",
                    "token": u"pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
                },
                "call_params": {
                    "media_connection_id": u"mc-102127d9-30de-413b-93f7-41a33e39d82b"
                },
            }
        )
        self.close_event = PeerEvent(
            {
                "event": "CLOSE",
                "params": {
                    "peer_id": u"hoge",
                    "token": u"pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
                },
            }
        )
        self.error_event = PeerEvent(
            {
                "event": "ERROR",
                "params": {
                    "peer_id": u"hoge",
                    "token": u"pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
                },
                "error_message": u"BROWSER_INCOMPATIBLE",
            }
        )

    def teardown_method(self, method):
        del self.subscribe_events
        del self.peer_info
        del self.event_sink
        del self.open_event
        del self.connection_event
        del self.call_event
        del self.close_event
        del self.error_event

    def test_status_request_params_succes(self, mocker):
        api_event_src = MagicMock()
        api_event_src.side_effect = [
            self.open_event,
            self.call_event,
            self.connection_event,
            self.close_event,
            self.call_event,
            self.error_event,
        ]
        mocker.patch("infra.peer.api.PeerApi.listen_event").side_effect = api_event_src

        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
            executor.submit(
                self.subscribe_events.run, self.peer_info, [self.event_sink],
            )
            assert self.event_sink.get() == self.open_event
            assert self.event_sink.get() == self.call_event
            assert self.event_sink.get() == self.connection_event
            assert self.event_sink.get() == self.close_event
            with pytest.raises(Queue.Empty):
                self.event_sink.get(timeout=0.1)

        executor.shutdown()

    def test_create_request_params_recv_error_event(self, mocker):
        api_event_src = MagicMock()
        api_event_src.side_effect = [
            self.error_event,
            self.open_event,
        ]
        mocker.patch("infra.peer.api.PeerApi.listen_event").side_effect = api_event_src

        with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
            executor.submit(
                self.subscribe_events.run, self.peer_info, [self.event_sink],
            )
            assert self.event_sink.get() == self.error_event
            with pytest.raises(Queue.Empty):
                self.event_sink.get(timeout=0.1)

        executor.shutdown()
