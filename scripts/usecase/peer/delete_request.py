# -*- coding: utf-8 -*-
import pinject

from domain.peer.interface import IPeerApi


class DeleteRequest:
    @pinject.annotate_arg("peer_api", "PeerApi")
    def __init__(self, peer_api):
        # type: (IPeerApi) -> None
        self.__api = peer_api

    def delete_request(self, peer_info):
        """
        A service that sends a request to remove a Peer Object.
        If you want to check when the peer is actually deleted,
        you need to monitor the CLOSE event.

        :param PeerInfo peer_info: Indicates which peer object to be deleted
        :return:
        :rtype: None
        """
        return self.__api.delete_request(peer_info)
