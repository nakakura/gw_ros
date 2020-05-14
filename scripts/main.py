#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os
import rospy
import pinject
import rosparam
import multiprocessing
import concurrent.futures
from std_msgs.msg import String

from helper.injector import BindingSpec
from usecase.peer.create_request import CreateRequest
from usecase.peer.delete_request import DeleteRequest
from presentation.ros_pub_sub import ros_subscriber, ros_publisher

PKG = "skyway"


def main():
    rospy.init_node("skyway", anonymous=True)

    peer_params = rospy.get_param("/skyway/peer")
    peer_params["key"] = unicode(os.environ["API_KEY"])
    peer_params["domain"] = unicode(peer_params["domain"])
    peer_params["peer_id"] = unicode(peer_params["peer_id"])

    rospy.logerr(peer_params)
    inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
    create_request = inject.provide(CreateRequest)
    result = create_request.create_request(peer_params)

    rospy.logerr(result)

    delete_request = inject.provide(DeleteRequest)
    delete_request.delete_request(result)

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
