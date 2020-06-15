# -*- coding: utf-8 -*-
import Queue
import pinject

from helper.injector import BindingSpec
from helper.methods import extract_target_item
from domain.common.model import DataConnectionId, DataId
from domain.data.model import RedirectParameters, Socket, Status
from usecase.data.status_request import StatusRequest
from usecase.data.redirect_request import RedirectRequest
from usecase.data.disconnect_request import DisconnectRequest
from usecase.data.open_data_socket_request import OpenDataSocketRequest


class RedirectFlow:
    def __init__(self):
        pass

    def run(self, original_config, data_connection_id):
        """
        redirects an DataConnection according to config
        :param list original_config:
        :param DataConnectionId data_connection_id:
        :return: result of redirect flow
        :rtype: dict
        """
        inject = pinject.new_object_graph(binding_specs=[BindingSpec()])

        # check metadata
        status_request = inject.provide(StatusRequest)
        status = status_request.run(data_connection_id)

        # load config
        (item, config) = extract_target_item(status.metadata, original_config)
        if item == {}:
            disconnect_request = inject.provide(DisconnectRequest)
            disconnect_request.run(data_connection_id)
            # if not have config -> reject(close socket, disconnect)
            return {"flag": False, "item": {}, "config": original_config}

        # if have config
        # open data sock
        open_socket_request = inject.provide(OpenDataSocketRequest)
        data_socket = open_socket_request.run()

        # redirect
        socket = {}
        redirect_params = item["redirect_params"]
        if "ip_v4" in redirect_params:
            socket = Socket(
                redirect_params["port"], ip_v4=redirect_params["ip_v4"].decode()
            )
        else:
            socket = Socket(
                redirect_params["port"], ip_v6=redirect_params["ip_v6"].decode()
            )

        params = RedirectParameters(data_socket.data_id(), socket)
        print params
        redirect_request = inject.provide(RedirectRequest)
        print "request"
        data_id = redirect_request.run(data_connection_id, params)
        print "before return"
        # show the message to user
        return {
            "flag": True,
            "data_socket": data_socket,
            "status": status,
            "item": item,
            "config": original_config,
        }
