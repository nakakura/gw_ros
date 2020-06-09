# -*- coding: utf-8 -*-
import pytest
import sys
import multiprocessing
import Queue
from os import path
import copy

sys.path.insert(
    0,
    path.dirname(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
    + "/scripts",
)
from helper.multi_queue import MultiQueue
from usecase.data.router import Router
from domain.data.model import DataControlEvents, DataControlEventType, Status
from domain.common.model import DataId, DataConnectionId


class TestRouter:
    def setup_method(self, method):
        self.peer_event_queue = multiprocessing.Queue()
        self.ros_event_queue = multiprocessing.Queue()
        self.event_sink = multiprocessing.Queue()
        self.data_connection_id = DataConnectionId(
            u"dc-102127d9-30de-413b-93f7-41a33e39d82b"
        )
        self.config = [
            {
                u"name": u"data",
                u"redirect_params": {u"ip_v4": u"127.0.0.1", u"port": 10000},
            },
            {
                u"name": u"data2",
                u"redirect_params": {u"ip_v6": u"fe00::1", u"port": 10001},
            },
        ]
        self.original_config = copy.deepcopy(self.config)
        self.router = Router(
            self.original_config,
            MultiQueue(self.ros_event_queue, self.peer_event_queue),
            self.event_sink,
        )
        self.connection_event = DataControlEvents(
            DataControlEventType.CONNECTION,
            {u"data_connection_id": self.data_connection_id.id()},
        )
        self.close_event = DataControlEvents(
            DataControlEventType.PEER_CLOSE,
            {
                u"peer_id": u"hoge",
                u"token": u"pt-870c2c49-c16d-4c69-b1ad-fec7550564af",
            },
        )
        self.status = Status(
            {
                u"remote_id": u"peer_id",
                u"buffersize": 0,
                u"label": u"",
                u"metadata": u"data",
                u"open": True,
                u"reliable": True,
                u"serialization": u"BINARY_UTF8",
                u"type": u"DATA",
            }
        )
        self.result = {
            u"type": u"DATA",
            u"data_params": {
                u"type": u"CONNECTION",
                u"data_connection_id": u"dc-102127d9-30de-413b-93f7-41a33e39d82b",
                u"socket": {
                    u"name": u"data",
                    u"redirect_params": {u"ip_v4": u"127.0.0.1", u"port": 10000},
                },
                u"status": {
                    u"remote_id": u"peer_id",
                    u"buffersize": 0,
                    u"label": u"",
                    u"metadata": u"data",
                    u"open": True,
                    u"reliable": True,
                    u"serialization": u"BINARY_UTF8",
                    u"type": u"DATA",
                },
            },
        }
        self.item = self.config.pop(0)
        self.return_value = {
            u"flag": True,
            u"data_id": DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211"),
            u"status": self.status,
            u"item": self.item,
            u"config": self.config,
        }

    def teardown_method(self, method):
        del self.return_value
        del self.item
        del self.original_config
        del self.router
        del self.peer_event_queue
        del self.data_connection_id
        del self.ros_event_queue
        del self.config
        del self.connection_event
        del self.close_event
        del self.result

    def test_connection_event(self, mocker):
        mock = mocker.patch("usecase.data.redirect_flow.RedirectFlow.run")

        mock.return_value = self.return_value

        self.peer_event_queue.put(self.connection_event)
        # exit from event loop
        self.peer_event_queue.put(self.close_event)
        self.router.run()

        assert mock.called
        assert mock.call_args_list == [
            mocker.call(self.original_config, self.data_connection_id)
        ]
        assert self.event_sink.get() == self.result
        with pytest.raises(Queue.Empty):
            self.event_sink.get(timeout=0.1)

    def test_connection_event_twice(self, mocker):
        mock = mocker.patch("usecase.data.redirect_flow.RedirectFlow.run")

        mock.side_effect = [
            self.return_value,
            {u"flag": False, u"item": {}, u"config": self.config,},
        ]

        self.peer_event_queue.put(self.connection_event)
        self.peer_event_queue.put(self.connection_event)
        # exit from event loop
        self.peer_event_queue.put(self.close_event)
        self.router.run()

        assert mock.called
        assert mock.call_args_list == [
            mocker.call(self.original_config, self.data_connection_id),
            mocker.call(self.config, self.data_connection_id),
        ]
        assert self.event_sink.get() == self.result
        with pytest.raises(Queue.Empty):
            self.event_sink.get(timeout=0.1)

    def test_connection_event_twice_nodata_and_success(self, mocker):
        mock = mocker.patch("usecase.data.redirect_flow.RedirectFlow.run")

        mock.side_effect = [
            {u"flag": False, u"item": {}, u"config": self.original_config,},
            self.return_value,
        ]

        self.peer_event_queue.put(self.connection_event)
        self.peer_event_queue.put(self.connection_event)
        # exit from event loop
        self.peer_event_queue.put(self.close_event)
        self.router.run()

        assert mock.called
        assert mock.call_args_list == [
            mocker.call(self.original_config, self.data_connection_id),
            mocker.call(self.original_config, self.data_connection_id),
        ]
        assert self.event_sink.get() == self.result
        with pytest.raises(Queue.Empty):
            self.event_sink.get(timeout=0.1)
