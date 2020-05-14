#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rosparam
import multiprocessing
import concurrent.futures

from presentation.ros_pub_sub import ros_subscriber, ros_publisher
from std_msgs.msg import String

PKG = "skyway"


def main():
    rospy.init_node("skyway", anonymous=True)

    event_sink = multiprocessing.Queue()
    executor = concurrent.futures.ThreadPoolExecutor(max_workers=2)
    executor.submit(ros_subscriber)
    executor.submit(ros_publisher, event_sink)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()

    executor.shutdown()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
