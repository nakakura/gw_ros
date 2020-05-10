# -*- coding: utf-8 -*-
from error import MyException


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


class PeerInfo:
    """
    Information for identifying PeerObject
    Fields
    peer_id: String, not null, an ID of Peer Object
    token: String, not null, UUID
    """

    def __init__(self, peer_id, token):
        # type: (str, str) -> None
        if not isinstance(peer_id, str) or len(peer_id) == 0:
            raise MyException("peer_id: invalid parameter in PeerInfo")
        self.__peer_id = peer_id

        if not isinstance(token, str) or len(token) == 0:
            raise MyException("token: invalid parameter in PeerInfo")
        self.__token = token

    def id(self):
        # type: () -> str
        return self.__peer_id

    def token(self):
        # type: () -> str
        return self.__token

    def __eq__(self, other):
        if not isinstance(other, PeerInfo):
            return NotImplemented
        return self.__peer_id == other.__peer_id and self.__token == other.__token
