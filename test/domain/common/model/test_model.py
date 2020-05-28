# -*- coding: utf-8 -*-
import pytest
import sys
from os import path

sys.path.insert(
    0,
    path.dirname(
        path.dirname(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
    )
    + "/scripts",
)
from error import MyException
from domain.common.model import PeerInfo, DataId, DataConnectionId


class TestPeerInfo:
    def setup_method(self, method):
        self.json = {
            "key": "key",
            "domain": "localhost",
            "peer_id": "my_id",
            "turn": True,
        }

    def teardown_method(self, method):
        del self.json

    @pytest.fixture(
        params=[("not_unicode_peer_id", "upt-102127d9-30de-413b-93f7-41a33e39d82b")]
    )
    def context(self, request):
        if request.param == "no_key":
            del self.json["key"]
            return self.json
        elif request.param == "blank_key":
            self.json["key"] = ""
            return self.json
        elif request.param == "no_domain":
            del self.json["domain"]
            return self.json
        elif request.param == "blank_domain":
            self.json["domain"] = ""
            return self.json
        elif request.param == "no_peer_id":
            del self.json["peer_id"]
            return self.json
        elif request.param == "blank_peer_id":
            self.json["peer_id"] = ""
            return self.json
        else:
            del self.json["turn"]
            return self.json

    def test_create_peer_info_success(self):
        peer_info = PeerInfo(u"my_id", u"pt-102127d9-30de-413b-93f7-41a33e39d82b")
        assert peer_info.id() == u"my_id", "peer_id is not correct"
        assert (
            peer_info.token() == u"pt-102127d9-30de-413b-93f7-41a33e39d82b"
        ), "token is not correct"

    @pytest.mark.parametrize(
        "peer_id, token",
        [
            ("not_unicode_id", u"pt-102127d9-30de-413b-93f7-41a33e39d82b"),
            (u"peer_id", "not_unicode_token"),
            (u"", u"pt-102127d9-30de-413b-93f7-41a33e39d82b"),
            (u"token_len_0", u""),
            (u"short_token", u"pt-102127d9-30de-413b-93f7-41a33e39d82"),
            (u"long_token", u"pt-102127d9-30de-413b-93f7-41a33e39d82b3"),
        ],
    )
    def test_create_peer_info_failed(self, peer_id, token):
        with pytest.raises(MyException):
            _peer_info = PeerInfo(peer_id, token)

    def test_compare_same_data_id(self):
        data_id = DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211")
        data_id2 = DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211")
        assert data_id == data_id2

    def test_compare_not_same_data_id(self):
        data_id = DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211")
        data_id2 = DataId(u"da-14a32bab-b3d9-4913-8e20-f79c90a6a211")
        assert data_id != data_id2

    @pytest.mark.parametrize(
        "data_id",
        [
            # not unicode
            "da-50a32bab-b3d9-4913-8e20-f79c90a6a211",
            # short
            u"da-50a32bab-b3d9-4913-8e20-f79c90a6a21",
            # long
            u"da-50a32bab-b3d9-4913-8e20-f79c90a6a2112",
            # wrong prefix
            u"dc-50a32bab-b3d9-4913-8e20-f79c90a6a211",
        ],
    )
    def test_create_data_id_fail(self, data_id):
        with pytest.raises(MyException):
            _data_id = DataId(data_id)

    def test_compare_same_data_connection_id(self):
        data_connection_id = DataConnectionId(
            u"dc-50a32bab-b3d9-4913-8e20-f79c90a6a211"
        )
        data_connection_id2 = DataConnectionId(
            u"dc-50a32bab-b3d9-4913-8e20-f79c90a6a211"
        )
        assert data_connection_id == data_connection_id2

    def test_compare_not_same_data_connection_id(self):
        data_connection_id = DataConnectionId(
            u"dc-11a32bab-b3d9-4913-8e20-f79c90a6a211"
        )
        data_connection_id2 = DataConnectionId(
            u"dc-50a32bab-b3d9-4913-8e20-f79c90a6a211"
        )
        assert data_connection_id != data_connection_id2

    @pytest.mark.parametrize(
        "data_connection_id",
        [
            # not unicode
            "dc-50a32bab-b3d9-4913-8e20-f79c90a6a211",
            # short
            u"dc-50a32bab-b3d9-4913-8e20-f79c90a6a21",
            # long
            u"dc-50a32bab-b3d9-4913-8e20-f79c90a6a2112",
            # wrong prefix
            u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",
        ],
    )
    def test_create_data_connection_id_fail(self, data_connection_id):
        with pytest.raises(MyException):
            _data_connection_id = DataConnectionId(data_connection_id)
