# -*- coding: utf-8 -*-
import pinject

from domain.peer.interface import IPeerApi


class StatusRequest:
    @pinject.annotate_arg("peer_api", "PeerApi")
    def __init__(self, peer_api):
        # type: (IPeerApi) -> None
        self.__api = peer_api

    def run(self, peer_info):
        """
        Request status of Peer Object
        Accessing GET /peers/{peer_id}/status API Internally
        http://35.200.46.204/#/1.peers/peer_status

        :param PeerInfo peer_info: Indicates which peer object to be deleted
        :return: Status of the PeerObject
        :rtype: PeerStatus
        """
        return self.__api.status_request(peer_info)
