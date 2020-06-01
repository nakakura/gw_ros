# -*- coding: utf-8 -*-
import pinject

from domain.data.interface import IDataApi
from domain.data.model import DataSocket, DataId


class CloseDataSocketRequest:
    @pinject.annotate_arg("data_api", "DataApi")
    def __init__(self, data_api):
        # type: (IDataApi) -> None
        self.__api = data_api

    def run(self, data_id):
        # type: (DataId) -> None
        self.__api.close_data_socket_request(data_id)
