# -*- coding: utf-8 -*-
import pinject

from infra.peer.api import PeerApi


class BindingSpec(pinject.BindingSpec):
    def configure(self, bind):
        pass

    @pinject.provides("peer_api", annotated_with="PeerApi")
    def provide_peer_api(self):
        return PeerApi("http://localhost:8000")
