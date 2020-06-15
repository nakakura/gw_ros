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
from helper.multi_queue import MultiQueue
from usecase.peer.create_request import CreateRequest
from usecase.peer.delete_request import DeleteRequest
from usecase.peer.subscribe_events import SubscribeEvents
from usecase.data.router import Router
from presentation.service import skyway_events_server, control_message_server
from domain.common.model import PeerInfo

PKG = "skyway"


def on_exit(peer_info):
    """
    this function delete peer object when ros is closing
    :param PeerInfo peer_info: identify which peer to close
    """
    rospy.spin()
    rospy.loginfo("shutting down")
    inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
    delete_request = inject.provide(DeleteRequest)
    try:
        delete_request.run(peer_info)
    except Exception as e:
        rospy.logerr(e)


def main():
    rospy.loginfo("initializing")
    rospy.init_node("skyway")

    # load config
    peer_config = rospy.get_param("/skyway/peer")
    data_config = rospy.get_param("/skyway/data")
    # API_KEY needs to be kept confidential,
    # so it's loaded from environment variables
    peer_config["key"] = os.environ["API_KEY"]

    # create services for Peer
    inject = pinject.new_object_graph(binding_specs=[BindingSpec()])
    create_request = inject.provide(CreateRequest)
    peer_event_subscribe_service = inject.provide(SubscribeEvents)

    # Notify PeerEvent Message to an End User Program
    queue_from_peer_event_to_presentation = multiprocessing.Queue()
    # Notify PeerEvent Message to the Router in UseCase
    queue_from_peer_event_to_usecase = multiprocessing.Queue()
    # Notify user message to the Router in Usecase
    queue_from_presentation_to_usecase = multiprocessing.Queue()
    # Notify usecase Event Messages to an End User Program
    queue_from_router_to_presentation = multiprocessing.Queue()

    router = Router(
        data_config,
        MultiQueue(
            queue_from_presentation_to_usecase, queue_from_peer_event_to_usecase
        ),
        queue_from_router_to_presentation,
    )
    # create a PeerObject
    peer_info = create_request.run(peer_config)

    rospy.loginfo("start process")
    with concurrent.futures.ThreadPoolExecutor(max_workers=8) as executor:
        # check if ROS is running
        # When ROS is closing, this method will delete the PeerObject
        executor.submit(on_exit, peer_info)

        # ----------UseCase----------
        # launch event listener from the PeerObject in WebRTC GW
        executor.submit(
            peer_event_subscribe_service.run,
            peer_info,
            [queue_from_peer_event_to_presentation, queue_from_peer_event_to_usecase],
        )
        # launch the router in UseCase
        executor.submit(router.run)
        # ----------UseCase----------

        # ----------Presentation----------
        # launch event notifier for end user program
        executor.submit(
            skyway_events_server,
            MultiQueue(
                queue_from_peer_event_to_presentation, queue_from_router_to_presentation
            ),
        )
        # launch message listener from end user program
        executor.submit(control_message_server, queue_from_presentation_to_usecase)
        # ----------Presentation----------

    executor.shutdown()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
