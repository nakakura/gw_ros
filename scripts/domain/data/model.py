# -*- coding: utf-8 -*-
from error import MyException


class DataId:
    def __init__(self, data_id):
        """
        a Value object of data_id
        :param unicode data_id: ID to identify the socket that will receive the data from the end user program
        """

        # TOKEN is prefix(da-, 3words) + UUID(36words) = 39words
        if (
            not isinstance(data_id, unicode)
            or len(data_id) != 39
            or not data_id.startswith("da-")
        ):
            raise MyException("invalid data_connection_id")
        self.__data_id = data_id

    def id(self):
        # type: () -> unicode
        return self.__data_id

    def __eq__(self, other):
        if not isinstance(other, DataId):
            return NotImplemented

        import rospy

        rospy.logerr("in eq {}, {}", self.id(), other.id())
        return self.id() == other.id()

    def __ne__(self, other):
        return not self.__eq__(other)


class DataConnectionId:
    def __init__(self, data_connection_id):
        # type: (unicode) -> None

        # TOKEN is prefix(dc-, 3words) + UUID(36words) = 39words
        if (
            not isinstance(data_connection_id, unicode)
            or len(data_connection_id) != 39
            or not data_connection_id.startswith("dc-")
        ):
            raise MyException("invalid media_connection_id")
        self.__data_connection_id = data_connection_id

    def id(self):
        # type: () -> unicode
        return self.__data_connection_id

    def __eq__(self, other):
        if not isinstance(other, DataConnectionId):
            return NotImplemented

        return self.id() == other.id()

    def __ne__(self, other):
        return not self.__eq__(other)
