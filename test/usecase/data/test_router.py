# -*- coding: utf-8 -*-
import pytest
import sys
import multiprocessing
from os import path
import copy

sys.path.insert(
    0,
    path.dirname(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
    + "/scripts",
)
from helper.multi_queue import MultiQueue
from usecase.data.router import Router
from domain.data.model import DataControlEvents, DataControlEventType
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
            {"name": "data", "redirect_params": {"ip_v4": "127.0.0.1", "port": 10000}},
            {"name": "data2", "redirect_params": {"ip_v6": "fe00::1", "port": 10001}},
        ]
        self.router = Router(
            self.config,
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

    def teardown_method(self, method):
        del self.router
        del self.peer_event_queue
        del self.data_connection_id
        del self.ros_event_queue
        del self.config
        del self.connection_event
        del self.close_event

    def test_connection_event(self, mocker):
        original_config = copy.deepcopy(self.config)
        config = copy.deepcopy(self.config)
        mock = mocker.patch("usecase.data.redirect_flow.RedirectFlow.run")
        item = config.pop(0)
        mock.return_value = {
            "flag": True,
            "data_id": DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211"),
            "item": item,
            "config": config,
        }

        self.peer_event_queue.put(self.connection_event)
        # exit from event loop
        self.peer_event_queue.put(self.close_event)
        self.router.run()

        assert mock.called
        assert mock.call_args_list == [
            mocker.call(original_config, self.data_connection_id)
        ]

    def test_connection_event_twice(self, mocker):
        original_config = copy.deepcopy(self.config)
        config = copy.deepcopy(self.config)
        mock = mocker.patch("usecase.data.redirect_flow.RedirectFlow.run")
        item = config.pop(0)

        mock.side_effect = [
            {
                "flag": True,
                "data_id": DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211"),
                "item": item,
                "config": config,
            },
            {
                "flag": True,
                "data_id": DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211"),
                "item": item,
                "config": config,
            },
        ]

        self.peer_event_queue.put(self.connection_event)
        self.peer_event_queue.put(self.connection_event)
        # exit from event loop
        self.peer_event_queue.put(self.close_event)
        self.router.run()

        assert mock.called
        assert mock.call_args_list == [
            mocker.call(original_config, self.data_connection_id),
            mocker.call(config, self.data_connection_id),
        ]

    def test_connection_event_twice_nodata_and_success(self, mocker):
        original_config = copy.deepcopy(self.config)
        config = copy.deepcopy(self.config)
        mock = mocker.patch("usecase.data.redirect_flow.RedirectFlow.run")
        item = config.pop(0)

        mock.side_effect = [
            {"flag": False, "item": {}, "config": original_config,},
            {
                "flag": True,
                "data_id": DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211"),
                "item": item,
                "config": config,
            },
        ]

        self.peer_event_queue.put(self.connection_event)
        self.peer_event_queue.put(self.connection_event)
        # exit from event loop
        self.peer_event_queue.put(self.close_event)
        self.router.run()

        assert mock.called
        assert mock.call_args_list == [
            mocker.call(original_config, self.data_connection_id),
            mocker.call(original_config, self.data_connection_id),
        ]
