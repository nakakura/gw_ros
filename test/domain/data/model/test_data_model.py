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
from domain.data.model import (
    Socket,
    DataSocket,
    DcInit,
    ConnectInnerOption,
    ConnectParameters,
    RedirectParameters,
    Status,
    DataControlEvents,
    DataControlEventType,
)
from domain.common.model import PeerInfo, DataId, DataConnectionId
from error import MyException


class TestSocket:
    def test_socket_compare_same(self):
        socket = Socket(10000, ip_v4=u"127.0.0.1")
        socket2 = Socket(10000, ip_v4=u"127.0.0.1")
        assert socket == socket2

    @pytest.fixture(
        params=[
            "different_ip_v4",
            "different_port_v4",
            "different_ip_v6",
            "different_port_v6",
            "different_stack",
        ]
    )
    def different_socket_context(self, request):
        if request.param == "different_ip_v4":
            return (u"127.0.0.1", u"", 10000), (u"127.0.0.2", u"", 10000)
        elif request.param == "different_port_v4":
            return (u"127.0.0.1", u"", 10000), (u"127.0.0.1", u"", 10001)
        elif request.param == "different_ip_v6":
            return (u"", u"fe00::1", 10000), (u"", u"fe00::2", 10000)
        elif request.param == "different_port_v6":
            return (u"", u"fe00::1", 10000), (u"", u"fe00::1", 10001)
        elif request.param == "different_stack":
            return (u"127.0.0.1", u"", 10000), (u"", u"fe00::1", 10002)

    def test_socket_compare_not_same(self, different_socket_context):
        ((ip_v4, ip_v6, port), (ip_v4_2, ip_v6_2, port_2)) = different_socket_context
        socket_obj = Socket(port, ip_v4=ip_v4, ip_v6=ip_v6)
        socket_obj2 = Socket(port_2, ip_v4=ip_v4_2, ip_v6=ip_v6_2)
        assert socket_obj != socket_obj2

    @pytest.fixture(
        params=[
            "invalid_port_lower_limit",
            "invalid_port_upper_limit",
            "invalid_port_type",
            "no_ip",
        ]
    )
    def invalid_socket_context(self, request):
        if request.param == "invalid_port_lower_limit":
            return u"127.0.0.1", u"", 0
        elif request.param == "invalid_port_upper_limit":
            return u"127.0.0.1", u"", 65536
        elif request.param == "invalid_port_type":
            return u"127.0.0.1", u"", "10000"
        elif request.param == "no_ip":
            return u"", u"", 10000

    def test_socket_create_fail(self, invalid_socket_context):
        (ip_v4, ip_v6, port) = invalid_socket_context
        with pytest.raises(MyException):
            _socket = Socket(port, ip_v4=ip_v4, ip_v6=ip_v6)


class TestDataSocket:
    def test_data_socket_compare_same(self):
        data_socket = DataSocket(
            u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",
            10000,
            ip_v4=u"127.0.0.1",
            ip_v6=u"",
        )
        data_socket2 = DataSocket(
            u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",
            10000,
            ip_v4=u"127.0.0.1",
            ip_v6=u"",
        )
        assert data_socket == data_socket2

    @pytest.fixture(
        params=["different_data_id",]
    )
    def different_socket_context(self, request):
        if request.param == "different_data_id":
            return (
                (u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211", u"127.0.0.1", u"", 10000),
                (u"da-50a32bab-b3d9-4913-8e20-f79c90a6a212", u"127.0.0.1", u"", 10000),
            )

    def test_data_socket_compare_different(self, different_socket_context):
        (
            (data_id, ip_v4, ip_v6, port),
            (data_id_2, ip_v4_2, ip_v6_2, port_2),
        ) = different_socket_context
        data_socket = DataSocket(data_id, port, ip_v4=ip_v4, ip_v6=ip_v6)
        data_socket_2 = DataSocket(data_id_2, port_2, ip_v4=ip_v4_2, ip_v6=ip_v6_2)

        assert data_socket != data_socket_2


class TestDcInit:
    def setup_method(self, method):
        self.json = {
            "ordered": True,
            "maxPacketLifeTime": 0,
            "maxRetransmits": 0,
            "protocol": "H264",
            "negotiated": True,
            "id": 0,
            "priority": "NONE",
        }

    def teardown_method(self, method):
        del self.json

    @pytest.fixture(
        params=[
            "full",
            "no_ordered",
            "no_lifetime",
            "no_retransmits",
            "no_protocol",
            "no_negotiated",
            "no_id",
            "no_priority",
        ]
    )
    def optional_json_context(self, request):
        if request.param == "full":
            return self.json
        elif request.param == "no_ordered":
            del self.json["ordered"]
            return self.json
        elif request.param == "no_lifetime":
            del self.json["maxPacketLifeTime"]
            return self.json
        elif request.param == "no_retransmits":
            del self.json["maxRetransmits"]
            return self.json
        elif request.param == "no_protocol":
            del self.json["protocol"]
            return self.json
        elif request.param == "no_negotiated":
            del self.json["negotiated"]
            return self.json
        elif request.param == "no_id":
            del self.json["id"]
            return self.json
        elif request.param == "no_priority":
            del self.json["priority"]
            return self.json

    def test_dcinit_json(self, optional_json_context):
        dc_init = DcInit(optional_json_context)
        assert dc_init.json() == optional_json_context

    def test_dcinit_compare_same(self):
        dc_init1 = DcInit(self.json)
        dc_init2 = DcInit(self.json)
        assert dc_init1 == dc_init2

    def test_dcinit_compare_not_same(self):
        dc_init1 = DcInit(self.json)
        self.json["ordered"] = False
        dc_init2 = DcInit(self.json)
        assert dc_init1 != dc_init2

    @pytest.fixture(
        params=[
            "ordered_not_bool",
            "lifetime_not_num",
            "retransmits_not_num",
            "protocol_not_str",
            "negotiated_not_bool",
            "id_not_num",
            "priority_not_str",
        ]
    )
    def invalid_dcinit_context(self, request):
        if request.param == "ordered_not_bool":
            self.json["ordered"] = 0
            return self.json
        elif request.param == "lifetime_not_num":
            self.json["maxPacketLifeTime"] = "0"
            return self.json
        elif request.param == "retransmits_not_num":
            self.json["maxRetransmits"] = "0"
            return self.json
        elif request.param == "protocol_not_str":
            self.json["protocol"] = 0
            return self.json
        elif request.param == "negotiated_not_bool":
            self.json["negotiated"] = "0"
            return self.json
        elif request.param == "id_not_num":
            self.json["id"] = "0"
            return self.json
        elif request.param == "priority_not_str":
            self.json["priority"] = 0
            return self.json

    def test_dcinit_create_fail(self, invalid_dcinit_context):
        with pytest.raises(MyException):
            _err = DcInit(invalid_dcinit_context)


class TestConnectInnerOption:
    def setup_method(self, method):
        self.json = {
            "metadata": "meta message",
            "serialization": "BINARY",
            "dcInit": {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            },
        }

    def teardown_method(self, method):
        del self.json

    def test_connect_inner_option_compare_same(self):
        inner_option = ConnectInnerOption(self.json)
        inner_option2 = ConnectInnerOption(self.json)
        assert inner_option == inner_option2

    def test_connect_inner_option_compare_not_same(self):
        del self.json["metadata"]
        inner_option = ConnectInnerOption(self.json)
        del self.json["serialization"]
        inner_option2 = ConnectInnerOption(self.json)
        assert inner_option != inner_option2

    @pytest.fixture(params=["invalid_metadata", "invalid_serialization"])
    def invalid_option_context(self, request):
        # type (str) -> dict
        if request.param == "invalid_metadata":
            self.json["metadata"] = 0
            return self.json
        elif request.param == "invalid_serialization":
            self.json["serialization"] = 0
            return self.json

    def test_connect_inner_option_create_fail(self, invalid_option_context):
        (json) = invalid_option_context
        with pytest.raises(MyException):
            _err = ConnectInnerOption(json)


class TestConnectParameters:
    def test_connect_parameters_compare_same(self):
        param = ConnectParameters(
            PeerInfo(u"peer_id", u"pt-9749250e-d157-4f80-9ee2-359ce8524308"),
            "target_id",
            DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",),
            Socket(10000, ip_v4=u"127.0.0.1"),
            options={
                "metadata": "string",
                "serialization": "string",
                "dcInit": {
                    "ordered": True,
                    "maxPacketLifeTime": 0,
                    "maxRetransmits": 0,
                    "protocol": "string",
                    "negotiated": True,
                    "id": 0,
                    "priority": "string",
                },
            },
        )

        param2 = ConnectParameters(
            PeerInfo(u"peer_id", u"pt-9749250e-d157-4f80-9ee2-359ce8524308"),
            "target_id",
            DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",),
            Socket(10000, ip_v4=u"127.0.0.1"),
            options={
                "metadata": "string",
                "serialization": "string",
                "dcInit": {
                    "ordered": True,
                    "maxPacketLifeTime": 0,
                    "maxRetransmits": 0,
                    "protocol": "string",
                    "negotiated": True,
                    "id": 0,
                    "priority": "string",
                },
            },
        )

        assert param == param2

    def test_connect_parameters_json(self):
        param = ConnectParameters(
            PeerInfo(u"peer_id", u"pt-9749250e-d157-4f80-9ee2-359ce8524308"),
            "target_id",
            DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",),
            Socket(10000, ip_v4=u"127.0.0.1"),
            options={
                "metadata": "string",
                "serialization": "string",
                "dcInit": {
                    "ordered": True,
                    "maxPacketLifeTime": 0,
                    "maxRetransmits": 0,
                    "protocol": "string",
                    "negotiated": True,
                    "id": 0,
                    "priority": "string",
                },
            },
        )

        assert param.json() == {
            "peer_id": u"peer_id",
            "token": u"pt-9749250e-d157-4f80-9ee2-359ce8524308",
            "options": {
                "metadata": "string",
                "serialization": "string",
                "dcInit": {
                    "ordered": True,
                    "maxPacketLifeTime": 0,
                    "maxRetransmits": 0,
                    "protocol": "string",
                    "negotiated": True,
                    "id": 0,
                    "priority": "string",
                },
            },
            "target_id": "target_id",
            "params": {"data_id": u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211"},
            "redirect_params": {"ip_v4": u"127.0.0.1", "port": 10000},
        }


class TestRedirectParams:
    def test_redirect_param_compare(self):
        param = RedirectParameters(
            DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",),
            Socket(10000, ip_v4=u"127.0.0.1"),
        )
        param2 = RedirectParameters(
            DataId(u"da-50a32bab-b3d9-4913-8e20-f79c90a6a211",),
            Socket(10000, ip_v4=u"127.0.0.1"),
        )
        assert param == param2


class TestStatus:
    def setup_method(self, method):
        self.json = {
            "remote_id": "ID_BAR",
            "buffersize": 0,
            "label": "string",
            "metadata": "metadata",
            "open": True,
            "reliable": True,
            "serialization": "BINARY_UTF8",
            "type": "DATA",
        }

    def teardown_method(self, method):
        del self.json

    def test_status_compare(self):
        param = Status(self.json)
        param2 = Status(self.json)

        assert param == param2

    def test_status_compare_not_same(self):
        param = Status(self.json)
        self.json["label"] = u"hoge"
        param2 = Status(self.json)

        assert param != param2

    @pytest.fixture(
        params=[
            "no_remote_id",
            "no_buffsize",
            "no_label",
            "no_metadata",
            "no_open",
            "no_reliable",
            "no_serialization",
            "no_type",
        ]
    )
    def invalid_status_context(self, request):
        if request.param == "no_remote_id":
            del self.json["remote_id"]
            return self.json
        elif request.param == "no_buffsize":
            del self.json["buffersize"]
            return self.json
        elif request.param == "no_label":
            del self.json["label"]
            return self.json
        elif request.param == "no_metadata":
            del self.json["metadata"]
            return self.json
        elif request.param == "no_open":
            del self.json["open"]
            return self.json
        elif request.param == "no_reliable":
            del self.json["reliable"]
            return self.json
        elif request.param == "no_serialization":
            del self.json["serialization"]
            return self.json
        elif request.param == "no_type":
            del self.json["type"]
            return self.json

    def test_status_create_fail(self, invalid_status_context):
        with pytest.raises(KeyError):
            _param = Status(self.json)


class TestDataControlEvents:
    def setup_method(self):
        self.peer_info = PeerInfo(
            u"peer_id", u"pt-9749250e-d157-4f80-9ee2-359ce8524308"
        )
        self.data_connection_id = DataConnectionId(
            u"dc-102127d9-30de-413b-93f7-41a33e39d82b"
        )

    def teardown_method(self):
        del self.peer_info
        del self.data_connection_id

    def test_peer_close(self):
        event = DataControlEvents(
            DataControlEventType.PEER_CLOSE,
            {u"peer_id": self.peer_info.id(), u"token": self.peer_info.token()},
        )

        assert event.type() == DataControlEventType.PEER_CLOSE
        assert event.peer_info() == self.peer_info
        with pytest.raises(AttributeError):
            event.data_connection_id()

    def test_connection(self):
        event = DataControlEvents(
            DataControlEventType.CONNECTION,
            {u"data_connection_id": self.data_connection_id.id()},
        )

        assert event.type() == DataControlEventType.CONNECTION
        assert event.data_connection_id() == self.data_connection_id
        with pytest.raises(AttributeError):
            event.peer_info()
