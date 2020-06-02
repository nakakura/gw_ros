from skyway.srv import *
import rospy


def skyway_events_response(queue):
    def inner(req):
        x = queue.get()
        return SkyWayEventsResponse(x)

    return inner


def skyway_events_server(queue):
    _s = rospy.Service("skyway_events", SkyWayEvents, skyway_events_response(queue))
    rospy.spin()
