# -*- coding: utf-8 -*-
import pinject

from domain.data.interface import IDataApi
from domain.data.model import ConnectParameters, Socket
from domain.common.model import PeerInfo, DataId, DataConnectionId


class ConnectRequest:
    @pinject.annotate_arg("data_api", "DataApi")
    def __init__(self, data_api):
        # type: (IDataApi) -> None
        self.__api = data_api

    def connect_request(self, peer_info, target_id, data_id, redirect_params, options):
        """
        Establish DataConnection to an other peer
        :param PeerInfo peer_info: Indicate which peer starts connecting
        :param str target_id: Indicate which peer to connect to
        :param DataId data_id: Indicates which data should be transferred to the neighbour
        :param Socket redirect_params: redirect received data to this socket
        :param dict options: options of DataConnection
        :return: data connection id
        :rtype: DataConnectionId
        """
        return self.__api.connect_request(
            ConnectParameters(
                peer_info, target_id, data_id, redirect_params, options=options
            )
        )
