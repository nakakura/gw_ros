# -*- coding: utf-8 -*-
import pinject

from domain.peer.interface import IPeerApi
from domain.peer.model import CreateRequestParams, PeerInfo


class CreateRequest:
    @pinject.annotate_arg("peer_api", "PeerApi")
    def __init__(self, peer_api):
        # type: (IPeerApi) -> None
        self.__api = peer_api

    def run(self, json):
        # type: (dict) -> PeerInfo
        params = CreateRequestParams(json)
        return self.__api.create_request(params)
