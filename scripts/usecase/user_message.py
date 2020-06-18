# -*- coding: utf-8 -*-


class UserMessage:
    def __init__(self, json):
        """
        Events from DataConnection
        :param dict json:
        """
        self.__type = json["type"]
        self.__json = json["value"]

    def type(self):
        return self.__type

    def json(self):
        return self.__json

    def __eq__(self, other):
        if not isinstance(other, UserMessage):
            return NotImplemented

        return self.json() == other.json()

    def __ne__(self, other):
        return not self.__eq__(other)
