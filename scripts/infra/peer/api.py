# -*- coding: utf-8 -*-
import requests
import simplejson

from domain.peer.interface import IPeerApi
from domain.peer.model import CreateRequestParams, PeerEvent, PeerInfo
from infra.rest import Rest


class PeerApi(IPeerApi):
    def __init__(self, domain):
        # type: (str) -> None
        self.__rest = Rest(domain)

    def create_request(self, param):
        # type: (CreateRequestParams) -> PeerInfo
        json = self.__rest.post("peers", param.json(), 201)
        return PeerInfo(json["params"]["peer_id"], json["params"]["token"])

    def listen_event(self, peer_info):
        """
        Get an event of PeerObject

        :param PeerInfo peer_info: Indicates which peer object to subscribe events
        :return: An event from PeerObject
        :rtype: PeerEvent
        """
        json = self.__rest.get(
            "peers/{}/events?token={}".format(peer_info.id(), peer_info.token()), 200
        )
        return PeerEvent(json)
