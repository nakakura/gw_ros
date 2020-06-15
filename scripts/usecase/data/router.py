# -*- coding: utf-8 -*-
from multiprocessing import Queue
from enum import Enum

from helper.multi_queue import MultiQueue
from domain.data.model import DataEventItem
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
            print event.json()
            print event.type()
            if event.type() == u"CONNECTION":
                print "connection"
                redirect_flow = RedirectFlow()
                print "connection2"
                data_connection_id = event.data_connection_id()
                print "connection3"
                params = redirect_flow.run(self.__config, data_connection_id)
                print params
                if params["flag"]:
                    self.__used_config.append(params["item"])
                    self.__config = params["config"]
                    result = {
                        u"type": DataResultType.CONNECTION.value,
                        u"data_socket": params["data_socket"].json(),
                        u"socket": params["item"],
                        u"status": params["status"].json(),
                    }
                    print "event sink put"
                    self.__event_sink.put(DataEventItem(ResultType.DATA.value, result))
                else:
                    continue
            elif event.type() == u"CLOSE":
                return
            elif event.type() == u"OPEN":
                continue
