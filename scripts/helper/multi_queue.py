# -*- coding: utf-8 -*-
import Queue
import rospy
from threading import Timer


class MultiQueue:
    def __init__(self, queue1, queue2):
        self.__queue1 = queue1
        self.__queue2 = queue2
        self.__loop_flag = True

    def generate(self):
        while not rospy.is_shutdown():
            try:
                yield self.__queue1.get(timeout=0.1)
            except Queue.Empty:
                pass

            try:
                yield self.__queue2.get(timeout=0.1)
            except Queue.Empty:
                pass

    def exit_loop(self):
        self.__loop_flag = False

    def get(self, timeout):
        t = Timer(timeout, self.exit_loop)
        t.start()
        offset = 0.1
        if offset > timeout / 2:
            offset = timeout / 2

        while self.__loop_flag:
            try:
                val = self.__queue1.get(timeout=offset)
                t.cancel()
                return val
            except Queue.Empty:
                pass

            try:
                val = self.__queue2.get(timeout=offset)
                t.cancel()
                return val
            except Queue.Empty:
                pass

        self.__loop_flag = True
        raise Queue.Empty
