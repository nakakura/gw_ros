# -*- coding: utf-8 -*-
import pinject

from infra.peer.api import PeerApi
from infra.data.api import DataApi


class BindingSpec(pinject.BindingSpec):
    def configure(self, bind):

        pass

    @pinject.provides("peer_api", annotated_with="PeerApi")
    def provide_peer_api(self):
        return PeerApi("http://localhost:8000")

    @pinject.provides("data_api", annotated_with="DataApi")
    def provide_data_api(self):
        return DataApi("http://localhost:8000")
