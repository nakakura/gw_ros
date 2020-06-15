# -*- coding: utf-8 -*-
import pinject

from domain.data.interface import IDataApi
from domain.data.model import RedirectParameters
from domain.common.model import DataConnectionId


class RedirectRequest:
    @pinject.annotate_arg("data_api", "DataApi")
    def __init__(self, data_api):
        # type: (IDataApi) -> None
        self.__api = data_api

    def run(self, data_connection_id, redirect_params):
        """
        Sets the redirect destination for the received data.
        Also, set information to indicate which data to send
        :param DataConnectionId data_connection_id: Indicate which data to send
        :param RedirectParameters redirect_params: redirect destination for the received data
        :return: data id
        :rtype: DataId
        """
        print "in run"
        return self.__api.redirect_request(data_connection_id, redirect_params)
