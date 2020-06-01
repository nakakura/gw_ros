# -*- coding: utf-8 -*-
import pinject

from domain.data.interface import IDataApi
from domain.common.model import DataConnectionId


class StatusRequest:
    @pinject.annotate_arg("data_api", "DataApi")
    def __init__(self, data_api):
        # type: (IDataApi) -> None
        self.__api = data_api

    def run(self, data_connection_id):
        """
        Shows status of DataConnection
        :param DataConnectionId data_connection_id: Indicates which DataConnection to show
        :return: status
        :rtype: Status
        """
        return self.__api.status_request(data_connection_id)
