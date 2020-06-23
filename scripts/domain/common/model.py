# -*- coding: utf-8 -*-
from error import MyException


class PeerInfo:
    """
    Information for identifying PeerObject
    Fields
    peer_id: String, not null, an ID of Peer Object
    token: String, not null, UUID
    """

    def __init__(self, peer_id, token):
        # type: (unicode, unicode) -> None
        if isinstance(peer_id, str):
            peer_id = peer_id.decode("utf-8")
        if not isinstance(peer_id, unicode) or len(peer_id) == 0:
            raise MyException("peer_id: invalid parameter in PeerInfo")
        self.__peer_id = peer_id

        if isinstance(token, str):
            token = token.decode("utf-8")
        # TOKEN is prefix(pt-, 3words) + UUID(36words) = 39words
        if (
            not isinstance(token, unicode)
            or len(token) != 39
            or not token.startswith("pt-")
        ):
            raise MyException("token: invalid parameter in PeerInfo")
        self.__token = token

    def id(self):
        # type: () -> unicode
        return self.__peer_id

    def token(self):
        # type: () -> unicode
        return self.__token

    def json(self):
        # type: () -> dict
        return {u"peer_id": self.__peer_id, u"token": self.__token}

    def __eq__(self, other):
        if not isinstance(other, PeerInfo):
            return NotImplemented
        return self.__peer_id == other.__peer_id and self.__token == other.__token

    def __ne__(self, other):
        return not self.__eq__(other)


class DataId:
    def __init__(self, data_id):
        """
        a Value object of data_id
        :param unicode data_id: ID to identify the socket that will receive the data from the end user program
        """

        if isinstance(data_id, str):
            data_id = data_id.decode("utf-8")
        # TOKEN is prefix(da-, 3words) + UUID(36words) = 39words
        if (
            not isinstance(data_id, unicode)
            or len(data_id) != 39
            or not data_id.startswith("da-")
        ):
            raise MyException("invalid data_id")
        self.__data_id = data_id

    def id(self):
        # type: () -> unicode
        return self.__data_id

    def __eq__(self, other):
        if not isinstance(other, DataId):
            return NotImplemented

        return self.id() == other.id()

    def __ne__(self, other):
        return not self.__eq__(other)


class DataConnectionId:
    def __init__(self, data_connection_id):
        # type: (unicode) -> None

        if isinstance(data_connection_id, str):
            data_connection_id = data_connection_id.decode("utf-8")

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
