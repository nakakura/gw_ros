# -*- coding: utf-8 -*-
import requests
import simplejson

from domain.data.model import DataSocket
from domain.data.interface import IDataApi
from infra.rest import Rest


class DataApi(IDataApi):
    def __init__(self, domain):
        # type: (str) -> None
        self.__rest = Rest(domain)

    def open_data_socket_request(self):
        """
        Have WebRTC Gateway open a socket to await outbound data
        :return: Port information
        :rtype: DataSocket
        """
        json = self.__rest.post("data", {}, 201)
        if u"ip_v4" in json:
            return DataSocket(json[u"data_id"], json[u"port"], ip_v4=json[u"ip_v4"])
        else:
            return DataSocket(json[u"data_id"], json[u"port"], ip_v6=json[u"ip_v6"])
