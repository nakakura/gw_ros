#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import rosparam

PKG = "skyway"


def main():
    rospy.init_node("skyway", anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
