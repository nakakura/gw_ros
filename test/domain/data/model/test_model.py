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
)
from domain.common.model import PeerInfo, DataId
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
    @pytest.fixture(
        params=["different_data_id",]
    )
    @pytest.mark.parametrize(
        "json",
        [
            # full
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            },
            # no_ordered
            {
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            },
            # no_maxPacketLifeTIme
            {
                "ordered": True,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            },
            # no maxRetransmits
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            },
            # no protocol
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            },
            # no negotiated
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "id": 0,
                "priority": "NONE",
            },
            # no id
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "priority": "NONE",
            },
            # no priority
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
            },
        ],
    )
    def test_dcinit_json(self, json):
        dc_init = DcInit(json)
        assert dc_init.json() == json

    def test_dcinit_compare_same(self):
        dc_init1 = DcInit(
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            },
        )
        dc_init2 = DcInit(
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            },
        )
        assert dc_init1 == dc_init2

    def test_dcinit_compare_not_same(self):
        dc_init1 = DcInit(
            {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            },
        )
        dc_init2 = DcInit(
            {
                "ordered": False,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            },
        )
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
            return {
                "ordered": 0,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            }
        elif request.param == "lifetime_not_num":
            return {
                "ordered": True,
                "maxPacketLifeTime": "0",
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            }
        elif request.param == "retransmits_not_num":
            return {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": "0",
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            }
        elif request.param == "protocol_not_str":
            return {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": 0,
                "negotiated": True,
                "id": 0,
                "priority": "NONE",
            }
        elif request.param == "negotiated_not_bool":
            return {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": 0,
                "id": 0,
                "priority": "NONE",
            }
        elif request.param == "id_not_num":
            return {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": "0",
                "priority": "NONE",
            }

        elif request.param == "priority_not_str":
            return {
                "ordered": True,
                "maxPacketLifeTime": 0,
                "maxRetransmits": 0,
                "protocol": "H264",
                "negotiated": True,
                "id": 0,
                "priority": 0,
            }

    def test_dcinit_create_fail(self, invalid_dcinit_context):
        with pytest.raises(MyException):
            _err = DcInit(invalid_dcinit_context)


# FIXME make parameters in before_all
class TestConnectInnerOption:
    def test_connect_inner_option_compare_same(self):
        inner_option = ConnectInnerOption(
            {
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
            },
        )
        inner_option2 = ConnectInnerOption(
            {
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
            },
        )
        assert inner_option == inner_option2

    def test_connect_inner_option_compare_not_same(self):
        inner_option = ConnectInnerOption(
            {
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
            },
        )
        inner_option2 = ConnectInnerOption(
            {
                "metadata": "meta message hogehoge",
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
            },
        )
        assert inner_option != inner_option2

    @pytest.fixture(params=["invalid_metadata", "invalid_serialization"])
    def invalid_option_context(self, request):
        # type (str) -> dict
        if request.param == "invalid_metadata":
            return {
                "metadata": "meta message hogehoge",
                "serialization": 0,
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
        elif request.param == "invalid_serialization":
            return {
                "metadata": "meta message hogehoge",
                "serialization": 0,
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

    def test_connect_inner_option_create_fail(self, invalid_option_context):
        (json) = invalid_option_context
        with pytest.raises(MyException):
            _err = ConnectInnerOption(json)


# FIXME make parameters in before_all
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
    def test_status_compare(self):
        param = Status(
            {
                "remote_id": "ID_BAR",
                "buffersize": 0,
                "label": "string",
                "metadata": "metadata",
                "open": True,
                "reliable": True,
                "serialization": "BINARY_UTF8",
                "type": "DATA",
            }
        )
        param2 = Status(
            {
                "remote_id": "ID_BAR",
                "buffersize": 0,
                "label": "string",
                "metadata": "metadata",
                "open": True,
                "reliable": True,
                "serialization": "BINARY_UTF8",
                "type": "DATA",
            }
        )

        assert param == param2
