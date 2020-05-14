def router(msg):
    # type (str) -> None
    import rospy

    rospy.logerr("recv msg: {}".format(msg))
    pass
