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
from usecase.peer.subscribe_events import SubscribeEvents
from presentation.ros_pub_sub import ros_subscriber, ros_publisher

PKG = "skyway"


def main():
    rospy.init_node("skyway", anonymous=True)
    # sink for events to be sent with ROS channel
    event_sink = multiprocessing.Queue()

    executor = concurrent.futures.ThreadPoolExecutor(max_workers=4)
    executor.submit(ros_subscriber)
    executor.submit(ros_publisher, event_sink)

    peer_params = rospy.get_param("/skyway/peer")
    peer_params["key"] = unicode(os.environ["API_KEY"])
    peer_params["domain"] = unicode(peer_params["domain"])
    peer_params["peer_id"] = unicode(peer_params["peer_id"])

    inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
    create_request = inject.provide(CreateRequest)
    peer_info = create_request.create_request(peer_params)

    subscribe_events = inject.provide(SubscribeEvents)
    event_future = executor.submit(
        subscribe_events.subscribe_events, peer_info, event_sink
    )

    rate = rospy.Rate(1)
    counter = 0
    while not rospy.is_shutdown():
        rate.sleep()
        counter += 1
        if counter > 5:
            delete_request = inject.provide(DeleteRequest)
            _result = delete_request.delete_request(peer_info)
            break

    try:
        delete_request = inject.provide(DeleteRequest)
        _result = delete_request.delete_request(peer_info)
    except Exception:
        # Peer Object may be already deleted
        pass
    rospy.signal_shutdown(0)
    event_future.result()
    executor.shutdown()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
