# -*- coding: utf-8 -*-
import pinject
import Queue
from enum import IntEnum
import rospy
import json as encoder

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
    def subscribe_events(self, peer_info, event_sink):
        """
        :param PeerInfo peer_info: Indicates which peer object to subscribe events
        :param event_sink: Subscriber of the events
        :return: None
        :rtype: None
        """

        while not rospy.is_shutdown():
            try:
                event = self.__api.listen_event(peer_info)
                event_sink.put(encoder.dumps(event.json()))
            except Exception as e:
                rospy.logerr(e)
            else:
                if event.type() == "CLOSE":
                    rate = rospy.Rate(1)
                    rate.sleep()
                    # if event is "CLOSE", the peer object has already been deleted.
                    # So terminate the subscription
                    break
                elif event.type() == "ERROR":
                    # if event is "ERROR", the peer object is something wrong.
                    # So terminate the subscription
                    break
