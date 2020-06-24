# -*- coding: utf-8 -*-
from multiprocessing import Queue
from enum import Enum

from helper.multi_queue import MultiQueue
from domain.data.model import DataEventItem
from usecase.data.connect_flow import ConnectFlow
from usecase.data.redirect_flow import RedirectFlow
from usecase.common import ResultType


class DataResultType(Enum):
    CONNECTION = u"CONNECTION"


class Router:
    def __init__(self, config, multi_queue, event_sink):
        """
        :param list config: Configurations for DataConnection
        :param MultiQueue multi_queue: Event Queues for PeerEvents and messages from ROS Service
        :param Queue event_sink: Event notifier
        """
        self.__config = config
        self.__used_config = []
        self.__queue = multi_queue
        self.__event_sink = event_sink

    def run(self):
        for event in self.__queue.generate():
            if event.type() == u"CONNECTION":
                redirect_flow = RedirectFlow()
                data_connection_id = event.data_connection_id()
                """
                returns JSON Value
                When Success
                param = {
                    "flag": True,
                    "data_socket": DataSocket,
                    "status": Status,
                    "item": dict,
                    "config": list(original_config),
                }
                When Fail
                param = {"flag": False, "item": {}, "config": list(original_config)}
                """
                params = redirect_flow.run(self.__config, data_connection_id)
                if params["flag"]:
                    # redirect success
                    self.__used_config.append(params["item"])
                    self.__config = params["config"]
                    result = {
                        u"type": DataResultType.CONNECTION.value,
                        u"data_socket": params["data_socket"].json(),
                        u"socket": params["item"],
                        u"status": params["status"].json(),
                    }
                    self.__event_sink.put(DataEventItem(ResultType.DATA.value, result))
                else:
                    # Abort the redirect
                    # This happens when there is no redirect information about the Connection
                    continue
            elif event.type() == u"CONNECT":
                connect_flow = ConnectFlow()
                config = event.json()
                val = connect_flow.run(config)
                if val["flag"]:
                    ret_value = {
                        u"type": u"CONNECTION",
                        u"data_socket": val["data_socket"].json(),
                        u"status": val["status"],
                    }
                    if "redirect_params" in config:
                        ret_value["socket"] = {
                            u"name": u"data",
                            u"redirect_params": config[u"redirect_params"],
                        }
                    self.__event_sink.put(
                        DataEventItem(ResultType.DATA.value, ret_value)
                    )
                else:
                    del val["flag"]
                    self.__event_sink.put(DataEventItem("CONNECTION_FAILED", val))
            elif event.type() == u"CLOSE":
                return
            elif event.type() == u"OPEN":
                continue
