# -*- coding: utf-8 -*-
import Queue
import pinject

from helper.injector import BindingSpec
from helper.methods import extract_target_item
from domain.common.model import PeerInfo, DataConnectionId, DataId
from domain.data.model import RedirectParameters, Socket, Status, ConnectParameters
from usecase.data.status_request import StatusRequest
from usecase.data.connect_request import ConnectRequest
from usecase.data.disconnect_request import DisconnectRequest
from usecase.data.open_data_socket_request import OpenDataSocketRequest
from error import MyException


class ConnectFlow:
    def __init__(self):
        pass

    def run(self, config):
        """

        :param dict config:
        :return:
        """
        inject = pinject.new_object_graph(binding_specs=[BindingSpec()])

        # parse config
        try:
            connect_parameters = ConnectParameters.from_json(config)
        except MyException as e:
            return {u"flag": False, u"error": e.message()}

        # create data socket
        open_socket_request = inject.provide(OpenDataSocketRequest)
        data_socket = open_socket_request.run()

        # connect
        connect_request = inject.provide(ConnectRequest)
        data_connection_id = connect_request.run(
            connect_parameters.peer_info(),
            connect_parameters.target_id(),
            data_socket.data_id,
            connect_parameters.redirect_params(),
            connect_parameters.options().json(),
        )

        # if success
        # get status
        status_request = inject.provide(StatusRequest)
        status = status_request.run(data_connection_id)

        # return value
        return {
            u"flag": True,
            u"socket": data_socket.socket().json(),
            u"status": status.json(),
        }
        # if fail -> return err
