# -*- coding: utf-8 -*-
import Queue
import rospy


class MultiQueue:
    def __init__(self, queue1, queue2):
        self.__queue1 = queue1
        self.__queue2 = queue2

    def generate(self):
        while not rospy.is_shutdown():
            try:
                yield self.__queue1.get(timeout=0.1)
            except Queue.Empty:
                pass

            try:
                yield self.__queue2.get(timeout=0.1)
            except Queue.Empty as e:
                raise e
