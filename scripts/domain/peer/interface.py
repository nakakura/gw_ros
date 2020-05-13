# -*- coding: utf-8 -*-
from abc import ABCMeta, abstractmethod

from model import CreateRequestParams, PeerEvent, PeerInfo


class IPeerApi:
    __metaclass__ = ABCMeta

    @abstractmethod
    def create_request(self, params):
        # type: (CreateRequestParams) -> PeerInfo
        pass

    @abstractmethod
    def listen_event(self, peer_info):
        """
        Get an event of PeerObject

        :param PeerInfo peer_info: Indicates which peer object to subscribe events
        :return: An event from PeerObject
        :rtype: PeerEvent
        """
        pass

    @abstractmethod
    def delete_request(self, peer_info):
        """
        Send a Delete Request of PeerObject
        Accessing DELETE /peer API Internally
        http://35.200.46.204/#/1.peers/peer_destroy

        :param PeerInfo peer_info: Indicates which peer object to be deleted
        :return:
        :rtype: None
        """
        pass

    @abstractmethod
    def status_request(self, peer_info):
        """
        Request status of Peer Object
        Accessing GET /peers/{peer_id}/status API Internally
        http://35.200.46.204/#/1.peers/peer_status

        :param PeerInfo peer_info: Indicates which peer object to be deleted
        :return: Status of the PeerObject
        :rtype: PeerStatus
        """
        pass
