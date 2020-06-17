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
        # peer_id is a mandatory field for the CONNECT API
        if "peer_id" not in config:
            return {u"flag": False, u"error": u"no_peer_id"}
        if "token" not in config:
            return {u"flag": False, u"error": u"no_token"}

        peer_info = PeerInfo(config["peer_id"], config["token"])
        socket = {}
        if "redirect_params" in config:
            redirect_params = {}
            if "ip_v4" in redirect_params:
                socket = Socket(redirect_params["port"], ip_v4=redirect_params["ip_v4"])
            else:
                socket = Socket(redirect_params["port"], ip_v6=redirect_params["ip_v6"])
        # create data socket
        open_socket_request = inject.provide(OpenDataSocketRequest)
        data_socket = open_socket_request.run()

        param = {}
        if "options" in config:
            param = config["options"]

        # connect
        connect_request = inject.provide(ConnectRequest)
        data_connection_id = connect_request.run(
            peer_info, config["target_id"], data_socket.data_id, socket, param
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
