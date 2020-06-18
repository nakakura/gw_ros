from skyway.srv import *
import rospy
import Queue
import simplejson as json
from simplejson import JSONDecodeError

from usecase.user_message import UserMessage
from helper.multi_queue import MultiQueue


def skyway_events_response(queue):
    """
    :param MultiQueue queue: Event Queue
    :return: Event or Timeout
    """

    def inner(req):
        timeout = 30.0
        try:
            param = json.loads(req.json)
            if "timeout" in param and isinstance(param["timeout"], float):
                timeout = param["timeout"]
        except JSONDecodeError as e:
            return SkyWayEventsResponse(
                json.dumps(
                    {
                        u"type": u"ERROR",
                        u"error_message": u"JSON_DECODE_ERROR",
                        u"detail": u"{}".format(e),
                    }
                )
            )

        try:
            # FIXME: filter events
            # CONNECTION from Peer should be filtered
            val = queue.get(timeout)
            return SkyWayEventsResponse(json.dumps(val.json()))
        except Queue.Empty:
            return SkyWayEventsResponse(json.dumps({u"type": u"TIMEOUT"}))

    return inner


def skyway_events_server(queue):
    """
    :param MultiQueue queue: Event Queue
    :return:
    """
    _s = rospy.Service("skyway_events", SkyWayEvents, skyway_events_response(queue))
    rospy.spin()


def control_message_response(queue):
    def inner(req):
        try:
            queue.put(UserMessage(json.loads(req.json)))
            return SkyWayControlsResponse('{"status": "accepted"}')
        except JSONDecodeError as e:
            return SkyWayEventsResponse(
                json.dumps(
                    {
                        u"type": u"ERROR",
                        u"error_message": u"JSON_DECODE_ERROR",
                        u"detail": u"{}".format(e),
                    }
                )
            )

    return inner


def control_message_server(queue):
    _s = rospy.Service(
        "skyway_control", SkyWayControls, control_message_response(queue)
    )
    rospy.spin()
