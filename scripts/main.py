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
    pass


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
