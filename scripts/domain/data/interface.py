# -*- coding: utf-8 -*-
from abc import ABCMeta, abstractmethod

from model import DataSocket, ConnectParameters, RedirectParameters
from domain.common.model import DataId, DataConnectionId


class IDataApi:
    __metaclass__ = ABCMeta

    @abstractmethod
    def open_data_socket_request(self):
        # type: () -> DataSocket
        pass

    @abstractmethod
    def close_data_socket_request(self, data_id):
        # type: (DataId) -> DataSocket
        pass

    @abstractmethod
    def connect_request(self, params):
        # type: (ConnectParameters) -> DataId
        pass

    @abstractmethod
    def disconnect_request(self, data_connection_id):
        # type: (DataConnectionId) -> None
        pass

    @abstractmethod
    def redirect_request(self, data_connection_id, redirect_params):
        # type: (DataConnectionId, RedirectParameters) -> DataId
        pass
