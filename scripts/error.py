# -*- coding: utf-8 -*-


class MyException(Exception):
    def __init__(self, value=None):
        # type (dict) -> None
        if value is None:
            self.__value = {}
        else:
            self.__value = value

    def __eq__(self, other):
        if not isinstance(other, MyException):
            return NotImplemented
        return self.__value == other.__value

    def __lt__(self, other):
        if not isinstance(other, MyException):
            return NotImplemented
        return self.__value < other.__value

    def __ne__(self, other):
        return not self.__eq__(other)

    def __le__(self, other):
        return self.__lt__(other) or self.__eq__(other)

    def __gt__(self, other):
        return not self.__le__(other)

    def __ge__(self, other):
        return not self.__lt__(other)
