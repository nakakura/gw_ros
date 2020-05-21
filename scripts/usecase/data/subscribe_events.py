# -*- coding: utf-8 -*-
import pinject
import rospy
import json as encoder

from error import MyException
from domain.data.interface import IDataApi
from domain.common.model import DataConnectionId
from domain.data.model import DataConnectionEventEnum


class StatusRequest:
    @pinject.annotate_arg("data_api", "DataApi")
    def __init__(self, data_api):
        # type: (IDataApi) -> None
        self.__api = data_api

    def subscribe_events(self, data_connection_id, event_sink):
        """
        Subscribe events from DataConnection
        :param DataConnectionId data_connection_id: Indicates which DataConection to subscribe events
        :param event_sink: Subscriber of the events
        :return: None
        :rtype: None
        """

        while not rospy.is_shutdown():
            try:
                event = self.__api.event(data_connection_id)
                event_sink.put(encoder.dumps(event.json()))
            except MyException as e:
                message = e.message()
                if message["status"] == 408:
                    continue
                else:
                    rospy.logerr("queue error in subscribe_events")
                    rospy.logerr(e)
            except Exception as e:
                rospy.logerr("queue error in subscribe_events")
                rospy.logerr(e)
            else:
                if event.type() == DataConnectionEventEnum.OPEN:
                    pass
                elif event.type() == DataConnectionEventEnum.CLOSE:
                    # if event is "CLOSE", the peer object has already been deleted.
                    # So terminate the subscription
                    break
                elif event.type() == DataConnectionEventEnum.ERROR:
                    # if event is "ERROR", the peer object is something wrong.
                    # So terminate the subscription
                    break
