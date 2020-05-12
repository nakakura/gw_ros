# -*- coding: utf-8 -*-
import pinject
import Queue
from enum import IntEnum

from domain.peer.interface import IPeerApi
from domain.peer.model import PeerInfo


class ControlEnum(IntEnum):
    APP_CLOSING = 0


class SubscribeEvents:
    @pinject.annotate_arg("peer_api", "PeerApi")
    def __init__(self, peer_api):
        # type: (IPeerApi) -> None
        self.__api = peer_api

    # This method subscribes events.
    #
    def subscribe_events(self, peer_info, control_src, event_sink):
        """
        :param PeerInfo peer_info: Indicates which peer object to subscribe events
        :param control_src: Controls runner in this method
        :param event_sink: Subscriber of the events
        :return: None
        :rtype: None
        """

        while True:
            try:
                message = control_src.get(timeout=0.1)
                if message["type"] == ControlEnum.APP_CLOSING:
                    break
            except Queue.Empty:
                # control_src usually doesn't hold a value
                pass
            except Exception as e:
                raise e

            event = self.__api.listen_event(peer_info)
            event_sink.put(event)
            if event.type() == "CLOSE":
                # if event is "CLOSE", the peer object has already been deleted.
                # So terminate the subscription
                break
            elif event.type() == "ERROR":
                # if event is "ERROR", the peer object is something wrong.
                # So terminate the subscription
                break
