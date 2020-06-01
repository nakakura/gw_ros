# -*- coding: utf-8 -*-
import pinject

from domain.data.interface import IDataApi
from domain.data.model import DataSocket


class OpenDataSocketRequest:
    @pinject.annotate_arg("data_api", "DataApi")
    def __init__(self, data_api):
        # type: (IDataApi) -> None
        self.__api = data_api

    def run(self):
        # type: () -> DataSocket
        return self.__api.open_data_socket_request()
