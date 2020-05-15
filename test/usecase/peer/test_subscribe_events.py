#!/usr/bin/env python
# -*- coding: utf-8 -*-
import logging
import sys
import unittest
from os import path
import rospy
import pinject
from mock import patch, MagicMock
import concurrent.futures
import Queue
import multiprocessing
import json as encoder

sys.path.append(
    path.dirname(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
)
sys.path.append(
    path.dirname(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
    + "/scripts"
)
from domain.peer.model import PeerEvent, PeerInfo
from usecase.peer.subscribe_events import SubscribeEvents, ControlEnum
from helper.injector import BindingSpec

PKG = "skyway"


class TestSubscribeEvents(unittest.TestCase):
    def test_create_request_params_success(self):
        inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
        event_subscriber = inject.provide(SubscribeEvents)
        peer_info = (PeerInfo(u"my_id", u"pt-102127d9-30de-413b-93f7-41a33e39d82b"),)
        event_sink = multiprocessing.Queue()
        api_event_src = MagicMock()
        open_event = PeerEvent(
            {
                "event": "OPEN",
                "params": {
                    "peer_id": u"hoge",
                    "token": u"pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
                },
            }
        )
        connection_event = PeerEvent(
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
        call_event = PeerEvent(
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
        close_event = PeerEvent(
            {
                "event": "CLOSE",
                "params": {
                    "peer_id": u"hoge",
                    "token": u"pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
                },
            }
        )
        call_event2 = PeerEvent(
            {
                "event": "CALL",
                "params": {
                    "peer_id": u"ThisEventWillNotBeReceived",
                    "token": u"pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
                },
                "call_params": {
                    "media_connection_id": u"mc-102127d9-30de-413b-93f7-41a33e39d82b"
                },
            }
        )
        api_event_src.side_effect = [
            open_event,
            call_event,
            connection_event,
            close_event,
            call_event2,
        ]

        with patch("infra.peer.api.PeerApi.listen_event", side_effect=api_event_src):
            with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
                executor.submit(
                    event_subscriber.subscribe_events, peer_info, event_sink,
                )
                self.assertEqual(event_sink.get(), encoder.dumps(open_event.json()))
                self.assertEqual(event_sink.get(), encoder.dumps(call_event.json()))
                self.assertEqual(
                    event_sink.get(), encoder.dumps(connection_event.json())
                )
                self.assertEqual(event_sink.get(), encoder.dumps(close_event.json()))
                with self.assertRaises(Queue.Empty):
                    event_sink.get(timeout=0.1)
            executor.shutdown()

    def test_create_request_params_recv_error_event(self):
        inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
        event_subscriber = inject.provide(SubscribeEvents)
        peer_info = (PeerInfo(u"my_id", u"pt-102127d9-30de-413b-93f7-41a33e39d82b"),)
        event_sink = multiprocessing.Queue()
        api_event_src = MagicMock()
        error_event = PeerEvent(
            {
                "event": "ERROR",
                "params": {
                    "peer_id": u"hoge",
                    "token": u"pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
                },
                "error_message": u"BROWSER_INCOMPATIBLE",
            }
        )
        open_event = PeerEvent(
            {
                "event": "OPEN",
                "params": {
                    "peer_id": u"hoge",
                    "token": u"pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
                },
            }
        )
        api_event_src.side_effect = [
            error_event,
            open_event,
        ]

        with patch("infra.peer.api.PeerApi.listen_event", side_effect=api_event_src):
            with concurrent.futures.ThreadPoolExecutor(max_workers=1) as executor:
                executor.submit(
                    event_subscriber.subscribe_events, peer_info, event_sink,
                )
                self.assertEqual(event_sink.get(), encoder.dumps(error_event.json()))
                with self.assertRaises(Queue.Empty):
                    event_sink.get(timeout=0.1)
            executor.shutdown()


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "data_api", TestSubscribeEvents)
