# -*- coding: utf-8 -*-
from error import MyException
from domain.data.model import DataConnectionId
from domain.common.model import PeerInfo


class CreateRequestParams:
    """
    This is body for Post /peers
    http://35.200.46.204/#/1.peers/peer
    Fields
    key: String, not null, SkyWay's API Key
    domain: String, not null, a domain registered with SkyWay that are bound to an API key
    peer_id: String, not null, an ID of Peer Object
    turn: bool, not null, Indicates whether this peer object wants to use the TURN server or not.
    """

    def __init__(self, json):
        # type (dict) -> None
        if "key" not in json:
            raise MyException("key: invalid parameter in CreateRequestParams")
        self.__key = json["key"]
        if not isinstance(self.__key, str) or len(self.__key) == 0:
            raise MyException("key: invalid parameter in CreateRequestParams")

        if "domain" not in json:
            raise MyException("domain: invalid parameter in CreateRequestParams")
        self.__domain = json["domain"]
        if not isinstance(self.__domain, str) or len(self.__domain) == 0:
            raise MyException("domain: invalid parameter in CreateRequestParams")

        if "peer_id" not in json:
            raise MyException("peer_id: invalid parameter in CreateRequestParams")
        self.__peer_id = json["peer_id"]
        if not isinstance(self.__peer_id, str) or len(self.__peer_id) == 0:
            raise MyException("peer_id: invalid parameter in CreateRequestParams")

        if "turn" not in json:
            raise MyException("turn : invalid parameter in CreateRequestParams")
        self.__turn = json["turn"]
        if not isinstance(self.__turn, bool):
            raise MyException("turn: invalid parameter in CreateRequestParams")

    def json(self):
        # type () -> dict
        return {
            "key": self.__key,
            "domain": self.__domain,
            "peer_id": self.__peer_id,
            "turn": self.__turn,
        }


class PeerEvent:
    def __init__(self, json):
        # type: (dict) -> None
        if "event" not in json:
            raise MyException("This json is not an event.")
        self.__event = json["event"]

        if self.__event == "OPEN":
            self.__peer_info = PeerInfo(
                json["params"]["peer_id"], json["params"]["token"]
            )
        elif self.__event == "CLOSE":
            self.__peer_info = PeerInfo(
                json["params"]["peer_id"], json["params"]["token"]
            )
        elif self.__event == "CALL":
            self.__peer_info = PeerInfo(
                json["params"]["peer_id"], json["params"]["token"]
            )
            self.__media_connection_id = MediaConnectionId(
                json["call_params"]["media_connection_id"]
            )
        elif self.__event == "CONNECTION":
            self.__peer_info = PeerInfo(
                json["params"]["peer_id"], json["params"]["token"]
            )
            self.__data_connection_id = DataConnectionId(
                json["data_params"]["data_connection_id"]
            )
        elif self.__event == "ERROR":
            self.__peer_info = PeerInfo(
                json["params"]["peer_id"], json["params"]["token"]
            )
            self.__error_message = json["error_message"]
        else:
            raise MyException("This json is not an peer event")

    def type(self):
        # type: () -> unicode
        return self.__event

    def peer_info(self):
        # type: () -> PeerInfo
        return self.__peer_info

    def media_connection_id(self):
        # type: () -> unicode
        return self.__media_connection_id.id()

    def data_connection_id(self):
        # type: () -> unicode
        return self.__data_connection_id.id()

    def error_message(self):
        # type: () -> unicode
        return self.__error_message

    def json(self):
        # type: () -> dict
        params = {}
        params["type"] = self.__event
        params["peer_id"] = self.peer_info().id()
        params["token"] = self.peer_info().token()
        if self.__event == "CALL":
            params["media_connection_id"] = self.media_connection_id()
        elif self.__event == "CONNECTION":
            params["data_connection_id"] = self.data_connection_id()
        elif self.__event == "ERROR":
            params["error"] = self.error_message()

        return params

    def __eq__(self, other):
        if not isinstance(other, PeerEvent):
            return NotImplemented

        if self.type() != other.type() or self.peer_info() != other.peer_info():
            return False

        if self.type() == "OPEN" or self.type() == "CLOSE":
            return True

        if self.type() == "CALL":
            return self.media_connection_id() == other.media_connection_id()

        if self.type() == "CONNECTION":
            return self.data_connection_id() == other.data_connection_id()

        if self.type() == "ERROR":
            return self.error_message() == other.error_message()

        return False

    def __ne__(self, other):
        return not self.__eq__(other)


class MediaConnectionId:
    def __init__(self, media_connection_id):
        # type: (unicode) -> None

        # TOKEN is prefix(mc-, 3words) + UUID(36words) = 39words
        if (
            not isinstance(media_connection_id, unicode)
            or len(media_connection_id) != 39
            or not media_connection_id.startswith("mc-")
        ):
            raise MyException("invalid media_connection_id")
        self.__media_connection_id = media_connection_id

    def id(self):
        # type: () -> unicode
        return self.__media_connection_id

    def __eq__(self, other):
        if not isinstance(other, MediaConnectionId):
            return NotImplemented

        return self.id() == other.id()

    def __ne__(self, other):
        return not self.__eq__(other)


class PeerStatus:
    def __init__(self, peer_id, disconnected):
        """
        Status of a PeerObject
        :param unicode peer_id: id of the PeerObject
        :param bool disconnected: shows the peer object is disconnected or not
        """

        if not isinstance(peer_id, unicode) or len(peer_id) == 0:
            raise MyException("peer_id: invalid parameter in PeerStatus")
        self.__peer_id = peer_id

        if not isinstance(disconnected, bool):
            raise MyException("disconnected: invalid parameter in PeerStatus")
        self.__disconnected = disconnected

    def id(self):
        # type: () -> unicode
        return self.__peer_id

    def disconnected(self):
        # type: () -> bool
        return self.__disconnected

    def __eq__(self, other):
        if not isinstance(other, PeerStatus):
            return NotImplemented
        return self.id() == other.id() and self.disconnected() == other.disconnected()

    def __ne__(self, other):
        return not self.__eq__(other)
