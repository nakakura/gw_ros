# -*- coding: utf-8 -*-
import Queue
import rospy


class QueueGenerator:
    def __init__(self, queue):
        self.__queue = queue

    def generate(self):
        # type() -> object
        while not rospy.is_shutdown():
            try:
                yield self.__queue.get(timeout=0.1)
            except Queue.Empty:
                pass
