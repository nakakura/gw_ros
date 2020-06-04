# -*- coding: utf-8 -*-
import pinject

from domain.data.interface import IDataApi
from domain.common.model import DataConnectionId


class DisconnectRequest:
    @pinject.annotate_arg("data_api", "DataApi")
    def __init__(self, data_api):
        # type: (IDataApi) -> None
        self.__api = data_api

    def run(self, data_connection_id):
        # type: (DataConnectionId) -> None
        self.__api.disconnect_request(data_connection_id)
