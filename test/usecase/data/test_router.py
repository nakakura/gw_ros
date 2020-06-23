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
from domain.data.model import (
    Status,
    DataSocket,
    DataEventItem,
)
from domain.common.model import DataId, DataConnectionId, PeerInfo
from domain.peer.model import PeerEvent
from usecase.user_message import UserMessage

# connect to neighbour
class TestRouterConnect:
    def setup_method(self, method):
        # setup queues
        self.peer_event_queue = multiprocessing.Queue()
        self.ros_event_queue = multiprocessing.Queue()
        self.event_sink = multiprocessing.Queue()
        # ---------- define parameters ----------
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
        self.peer_info = PeerInfo(u"hoge", u"pt-870c2c49-c16d-4c69-b1ad-fec7550564af")
        self.redirect_params = {u"ip_v4": u"127.0.0.1", u"port": 10000}
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
        self.close_event = PeerEvent(
            {"event": "CLOSE", "params": self.peer_info.json()}
        )
        self.data_id = DataId(u"da-102127d9-30de-413b-93f7-41a33e39d82b")
        self.user_message = {
            u"type": u"CONNECT",
            u"value": {
                u"peer_id": self.peer_info.id(),
                u"token": self.peer_info.token(),
                u"options": {
                    u"metadata": "string",
                    u"serialization": "string",
                    u"dcInit": {
                        u"ordered": True,
                        u"maxPacketLifeTime": 0,
                        u"maxRetransmits": 0,
                        u"protocol": "H264",
                        u"negotiated": True,
                        u"id": 0,
                        u"priority": "HIGH",
                    },
                },
                u"target_id": u"ID_BAR",
                u"redirect_params": self.redirect_params,
            },
        }
        # ---------- setup router ----------
        self.router = Router(
            self.original_config,
            MultiQueue(self.ros_event_queue, self.peer_event_queue),
            self.event_sink,
        )
        # ---------- define result ----------
        self.result = {
            u"type": u"CONNECTION",
            u"data_socket": DataSocket(
                self.data_id.id(), 10000, ip_v4=u"127.0.0.1"
            ).json(),
            u"socket": {u"name": u"data", u"redirect_params": self.redirect_params},
            u"status": self.status.json(),
        }

    def teardown_method(self, method):
        del self.result
        del self.router
        del self.user_message
        del self.data_id
        del self.close_event
        del self.status
        del self.redirect_params
        del self.peer_info
        del self.original_config
        del self.config
        del self.event_sink
        del self.ros_event_queue
        del self.peer_event_queue

    def test_connect_method(self, mocker):
        result = DataEventItem(u"CONNECTION", self.result)

        # ---------- set events ----------
        message = UserMessage(self.user_message)
        self.ros_event_queue.put(message)
        self.peer_event_queue.put(self.close_event)

        # ---------- run ----------
        mock = mocker.patch("usecase.data.connect_flow.ConnectFlow.run")
        mock.return_value = {
            u"flag": True,
            u"socket": self.redirect_params,
            u"data_socket": DataSocket(self.data_id.id(), 10000, ip_v4=u"127.0.0.1"),
            u"status": self.status.json(),
        }
        mock_open_sock = mocker.patch("infra.data.api.DataApi.open_data_socket_request")
        mock_open_sock.return_value = DataSocket(
            self.data_id.id(), 10001, ip_v4=u"127.0.0.1",
        )

        self.router.run()

        # ---------- evaluate ----------
        assert self.event_sink.get(timeout=0.1) == result
        with pytest.raises(Queue.Empty):
            self.event_sink.get(timeout=0.1)

    def test_connect_method_without_redirect(self, mocker):
        del self.result["socket"]
        result = DataEventItem(u"CONNECTION", self.result)

        # ---------- set events ----------
        del self.user_message["value"]["redirect_params"]
        message = UserMessage(self.user_message)
        self.ros_event_queue.put(message)
        self.peer_event_queue.put(self.close_event)

        # ---------- run ----------
        mock = mocker.patch("usecase.data.connect_flow.ConnectFlow.run")
        mock.return_value = {
            u"flag": True,
            u"data_socket": DataSocket(self.data_id.id(), 10000, ip_v4=u"127.0.0.1"),
            u"status": self.status.json(),
        }
        mock_open_sock = mocker.patch("infra.data.api.DataApi.open_data_socket_request")
        mock_open_sock.return_value = DataSocket(
            self.data_id.id(), 10001, ip_v4=u"127.0.0.1",
        )

        self.router.run()

        # ---------- evaluate ----------
        assert self.event_sink.get(timeout=0.1) == result
        with pytest.raises(Queue.Empty):
            self.event_sink.get(timeout=0.1)

    def test_connect_method_err(self, mocker):
        result = DataEventItem(u"CONNECTION_FAILED", {"error": "ERROR MESSAGE"})

        # ---------- set events ----------
        message = UserMessage(self.user_message)
        self.ros_event_queue.put(message)
        self.peer_event_queue.put(self.close_event)

        # ---------- run ----------
        mock = mocker.patch("usecase.data.connect_flow.ConnectFlow.run")
        mock.return_value = {u"flag": False, u"error": "ERROR MESSAGE"}
        mock_open_sock = mocker.patch("infra.data.api.DataApi.open_data_socket_request")
        mock_open_sock.return_value = DataSocket(
            self.data_id.id(), 10001, ip_v4=u"127.0.0.1",
        )

        self.router.run()

        # ---------- evaluate ----------
        assert self.event_sink.get(timeout=0.1) == result
        with pytest.raises(Queue.Empty):
            self.event_sink.get(timeout=0.1)


# connect from neighbour
class TestRouterConnection:
    def setup_method(self, method):
        self.peer_info = PeerInfo(u"hoge", u"pt-870c2c49-c16d-4c69-b1ad-fec7550564af")
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
        self.connection_event = PeerEvent(
            {
                "event": "CONNECTION",
                "params": self.peer_info.json(),
                "data_params": {"data_connection_id": self.data_connection_id.id()},
            }
        )

        self.close_event = PeerEvent(
            {"event": "CLOSE", "params": self.peer_info.json()}
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
        result = {
            u"type": u"CONNECTION",
            u"data_socket": DataSocket(
                u"da-102127d9-30de-413b-93f7-41a33e39d82b", 10000, ip_v4=u"127.0.0.1"
            ).json(),
            u"socket": {
                u"name": u"data",
                u"redirect_params": {u"ip_v4": u"127.0.0.1", u"port": 10000},
            },
            u"status": self.status.json(),
        }
        self.result = DataEventItem(u"CONNECTION", result)
        self.item = self.config.pop(0)
        self.return_value = {
            u"flag": True,
            u"data_socket": DataSocket(
                u"da-102127d9-30de-413b-93f7-41a33e39d82b", 10000, ip_v4=u"127.0.0.1",
            ),
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
        del self.peer_info

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
