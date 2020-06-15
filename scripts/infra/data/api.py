# -*- coding: utf-8 -*-
import requests
import simplejson

from domain.data.model import (
    DataSocket,
    ConnectParameters,
    RedirectParameters,
    DataConnectionEvent,
    Status,
)
from domain.common.model import DataId, DataConnectionId
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

    def close_data_socket_request(self, data_id):
        """
        Have WebRTC Gateway close the socket
        :param DataId data_id:
        :return:
        """
        self.__rest.delete("data/{}".format(data_id.id()), 204)

    def connect_request(self, params):
        """
        Establish DataConnection to an other peer
        :param ConnectParameters params:
        :return: data connection id
        :rtype: DataConnectionId
        """
        json = self.__rest.post("data/connections", params.json(), 202)
        return DataConnectionId(json["params"]["data_connection_id"])

    def disconnect_request(self, data_connection_id):
        """
        Disconnect DataConnection
        :param DataConnectionId data_connection_id:
        """
        self.__rest.delete("data/connections/{}".format(data_connection_id.id()), 204)

    def redirect_request(self, data_connection_id, redirect_params):
        """
        Sets the redirect destination for the received data.
        Also, set information to indicate which data to send
        :param DataConnectionId data_connection_id: Identify which DataConnection to configure
        :param RedirectParameters redirect_params: Configure Parameter
        :return: data id
        :rtype: DataId
        """
        print "redirect request"
        print redirect_params
        print redirect_params.json()
        json = self.__rest.put(
            "data/connections/{}".format(data_connection_id.id()),
            redirect_params.json(),
            200,
        )
        print json
        return DataId(json["data_id"])

    def event(self, data_connection_id):
        """
        Get Events of DataConnection
        :param DataConnectionId data_connection_id: Indicate which DataConnection to get event
        :return: event
        :rtype: DataConnectionEvent
        """
        json = self.__rest.get(
            "data/connections/{}/events".format(data_connection_id.id()), 200
        )
        return DataConnectionEvent(json)

    def status_request(self, data_connection_id):
        """
        Shows status of DataConnection
        :param DataConnectionId data_connection_id: Indicates which DataConnection to show
        :return: status
        :rtype: Status
        """
        json = self.__rest.get(
            "data/connections/{}/status".format(data_connection_id.id()), 200
        )
        return Status(json)
