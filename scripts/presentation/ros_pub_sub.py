# -*- coding: utf-8 -*-
import rospy
import Queue
import multiprocessing
import rospy

from std_msgs.msg import String


def ros_publisher(display_src):
    """
    Notify messages with ROS Channel.
    Messages are json format ROS Strings.
    :param multiprocessing.Queue display_src: message objects
    """

    rate = rospy.Rate(10)
    pub = rospy.Publisher("gw_events", String, queue_size=1)

    while not rospy.is_shutdown():
        try:
            message = display_src.get(timeout=0.1)
            pub.publish(String(message))
            rate.sleep()
        except Queue.Empty:
            # Ne message to notify. Skip
            pass


def ros_subscriber():
    """
    Observe messages from Controller with ROS Channel.
    Events that this function receives are redirects to usecase.router
    """

    rospy.Subscriber("gw_control_messages", String, redirect_message)
    rospy.spin()


def redirect_message(message):
    """
    unwrap std_msgs.msg and redirect inner message to router in usecase
    :param String message:
    """
    from usecase.router import router

    router(message.data)
