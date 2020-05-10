# -*- coding: utf-8 -*-
from abc import ABCMeta, abstractmethod

from model import CreateRequestParams, PeerInfo


class IPeerApi:
    __metaclass__ = ABCMeta

    @abstractmethod
    def create_peer(self, params):
        # type: (CreateRequestParams) -> PeerInfo
        pass
