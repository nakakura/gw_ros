# -*- coding: utf-8 -*-
import requests
import simplejson

from domain.peer.interface import IPeerApi
from domain.peer.model import CreateRequestParams, PeerInfo


class PeerApi(IPeerApi):
    # FIXME: skeleton
    def create_peer(self, param):
        # type: (CreateRequestParams) -> PeerInfo
        pass
