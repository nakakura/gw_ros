# -*- coding: utf-8 -*-
from enum import IntEnum

from error import MyException
from domain.common.model import DataId, DataConnectionId, PeerInfo


class Socket:
    def __init__(self, port, ip_v4=u"", ip_v6=u""):
        """
        Socket Info. a Pair of address and port
        :param int port:
        :param unicode ip_v4:
        :param unicode ip_v6:
        """

        if not isinstance(port, int) or port <= 0 or port > 65535:
            raise MyException("invalid port num")
        self.__port = port

        if not isinstance(ip_v4, unicode) or not isinstance(ip_v6, unicode):
            raise MyException("invalid address info")

        if len(ip_v4) == 0 and len(ip_v6) == 0:
            raise MyException("no address info")

        if len(ip_v4) != 0:
            self.__is_ip_v4 = True
        else:
            self.__is_ip_v4 = False
        self.__ip_v4 = ip_v4
        self.__ip_v6 = ip_v6

    def port(self):
        """
        Get Port Num
        :return: port num
        :rtype: int
        """
        return self.__port

    def is_ip_v4(self):
        """
        Indicates this object includes IPv4 addr or not
        :return: True if it includes IPv4 addr
        :rtype: bool
        """
        return self.__is_ip_v4

    def ip_v4(self):
        """
        Get IPv4 addr
        :return: address
        :rtype: unicode
        """
        return self.__ip_v4

    def ip_v6(self):
        """
        Get IPv6 addr
        :return: address
        :rtype: unicode
        """
        return self.__ip_v6

    def json(self):
        """
        return JSON value of thie object
        :return: json
        :rtypr: dict
        """
        json = {"port": self.port()}
        if self.is_ip_v4():
            json["ip_v4"] = self.ip_v4()
        else:
            json["ip_v6"] = self.ip_v6()
        return json

    def __eq__(self, other):
        if not isinstance(other, Socket):
            return NotImplemented

        return (
            self.port() == other.port()
            and self.is_ip_v4() == other.is_ip_v4()
            and self.ip_v4() == other.ip_v4()
            and self.ip_v6() == other.ip_v6()
        )

    def __ne__(self, other):
        return not self.__eq__(other)


class DataSocket:
    def __init__(self, data_id, port, ip_v4=u"", ip_v6=u""):
        self.__data_id = DataId(data_id)
        self.__socket = Socket(port, ip_v4=ip_v4, ip_v6=ip_v6)

    def socket(self):
        """
        Socket Info
        :return: Socket
        :rtype: Socket
        """
        return self.__socket

    def data_id(self):
        """
        Data Id
        :return: Data Id
        :rtype: DataId
        """
        return self.__data_id

    def json(self):
        return {u"data_id": self.__data_id.id(), u"socket": self.__socket.json()}

    def __eq__(self, other):
        if not isinstance(other, DataSocket):
            return NotImplemented

        return self.data_id() == other.data_id() and self.socket() == other.socket()

    def __ne__(self, other):
        return not self.__eq__(other)


class DcInit:
    def __init__(self, dict):
        """
        DcInit Parameter
        :param dict dict:
        """
        if "ordered" in dict:
            self.ordered = dict["ordered"]
            if not isinstance(self.ordered, bool):
                raise MyException("invalid parameter of ordered in DcInit")

        if "maxPacketLifeTime" in dict:
            self.maxPacketLifeTime = dict["maxPacketLifeTime"]
            if not isinstance(self.maxPacketLifeTime, int):
                raise MyException("invalid parameter of maxPacketLifeTime in DcInit")

        if "maxRetransmits" in dict:
            self.maxRetransmits = dict["maxRetransmits"]
            if not isinstance(self.maxRetransmits, int):
                raise MyException("invalid parameter of maxRetransmits in DcInit")

        if "protocol" in dict:
            self.protocol = dict["protocol"]
            if not isinstance(self.protocol, str):
                raise MyException("invalid parameter of protocol in DcInit")

        if "negotiated" in dict:
            self.negotiated = dict["negotiated"]
            if not isinstance(self.negotiated, bool):
                raise MyException("invalid parameter of negotiated in DcInit")

        if "id" in dict:
            self.id = dict["id"]
            if not isinstance(self.id, int):
                raise MyException("invalid parameter of id in DcInit")

        if "priority" in dict:
            self.priority = dict["priority"]
            if not isinstance(self.priority, str):
                raise MyException("invalid parameter of priority in DcInit")

    def json(self):
        """
        return parameters as JSON
        :return: json
        :rtype: dict
        """
        json = {}
        if hasattr(self, "ordered"):
            json["ordered"] = self.ordered

        if hasattr(self, "maxPacketLifeTime"):
            json["maxPacketLifeTime"] = self.maxPacketLifeTime

        if hasattr(self, "maxRetransmits"):
            json["maxRetransmits"] = self.maxRetransmits

        if hasattr(self, "protocol"):
            json["protocol"] = self.protocol

        if hasattr(self, "negotiated"):
            json["negotiated"] = self.negotiated

        if hasattr(self, "id"):
            json["id"] = self.id

        if hasattr(self, "priority"):
            json["priority"] = self.priority

        return json

    def __eq__(self, other):
        if not isinstance(other, DcInit):
            return NotImplemented

        return self.json() == other.json()

    def __ne__(self, other):
        return not self.__eq__(other)


class ConnectInnerOption:
    def __init__(self, json):
        """
        Internal Options of ConnectOption
        :param dict json:
        """
        if "metadata" in json:
            self.metadata = json["metadata"]
            if not isinstance(self.metadata, str):
                raise MyException("invalid parameter of metadata in ConnectInnerOption")

        if "serialization" in json:
            self.serialization = json["serialization"]
            if not isinstance(self.serialization, str):
                raise MyException(
                    "invalid parameter of serialization in ConnectInnerOption"
                )

        if "dcInit" in json:
            self.dcInit = DcInit(json["dcInit"])

    def json(self):
        """
        return parameters as JSON
        :return: json
        :rtype: dict
        """

        json = {}
        if hasattr(self, "metadata"):
            json["metadata"] = self.metadata

        if hasattr(self, "serialization"):
            json["serialization"] = self.serialization

        if hasattr(self, "dcInit"):
            json["dcInit"] = self.dcInit.json()

        return json

    def __eq__(self, other):
        if not isinstance(other, ConnectInnerOption):
            return NotImplemented

        return self.json() == other.json()

    def __ne__(self, other):
        return not self.__eq__(other)


class ConnectParameters:
    def __init__(
        self, peer_info, target_id, data_id=None, redirect_params=None, options=None
    ):
        """
        Body of POST /data/connections
        http://35.200.46.204/#/2.data/data_connections_create

        :param PeerInfo peer_info:
        :param strtarget_id:
        :param DataId data_id:
        :param Socket redirect_params:
        :param dict options:
        """
        if options is None:
            options = {}
        self.__peer_info = peer_info
        self.__target_id = target_id
        self.__data_id = data_id
        self.__redirect_params = redirect_params
        print options
        import rospy

        rospy.logerr(options)
        self.__options = ConnectInnerOption(options)

    @staticmethod
    def from_json(params):
        """
        :param dict params:
        :return: ConnectParameters
        :rtype: ConnectParameters
        """
        if not "peer_id" in params:
            raise MyException("no peer_id")
        if not "token" in params:
            raise MyException("no token")
        if not "target_id" in params:
            raise MyException("no target_id")

        return ConnectParameters(
            PeerInfo(params["peer_id"], params["token"]),
            params["target_id"],
            data_id=params.get("data_id"),
            redirect_params=params.get("redirect_params"),
            options=params.get("options"),
        )

    def json(self):
        """
        return parameters as JSON
        :return: json
        :rtype: dict
        """
        param = {
            "peer_id": self.__peer_info.id(),
            "token": self.__peer_info.token(),
            "target_id": self.__target_id,
            "options": self.__options.json(),
        }
        if not self.__data_id is None:
            param["params"] = {"data_id": self.__data_id.id()}
        if not self.__redirect_params is None:
            param["redirect_params"] = self.__redirect_params.json()

        return param

    def peer_info(self):
        return self.__peer_info

    def target_id(self):
        return self.__target_id

    def redirect_params(self):
        return self.__redirect_params

    def options(self):
        return self.__options

    def __eq__(self, other):
        if not isinstance(other, ConnectParameters):
            return NotImplemented

        return self.json() == other.json()

    def __ne__(self, other):
        return not self.__eq__(other)


class RedirectParameters:
    def __init__(self, data_id, redirect_params):
        """
        Sets the redirect destination for the received data.
        Also, set information to indicate which data to send
        :param DataId data_id: Indicate which data to send
        :param Socket redirect_params: redirect destination for the received data
        """
        self.__data_id = data_id
        self.__redirect_params = redirect_params

    def json(self):
        """
        return parameters as JSON
        :return: json
        :rtype: dict
        """

        print "hoge"
        print self.__data_id
        print self.__data_id.id()
        print self.__redirect_params.json()
        return {
            "feed_params": {"data_id": self.__data_id.id()},
            "redirect_params": self.__redirect_params.json(),
        }

    def __eq__(self, other):
        if not isinstance(other, RedirectParameters):
            return NotImplemented

        return self.json() == other.json()

    def __ne__(self, other):
        return not self.__eq__(other)


class DataConnectionEventEnum(IntEnum):
    OPEN = 0
    CLOSE = 1
    ERROR = -1


class DataConnectionEvent:
    def __init__(self, json):
        """
        Events from DataConnection
        :param dict json:
        """
        if "event" not in json:
            raise MyException("invalid datatype of DataConnectionEvent")

        if json["event"] == u"OPEN":
            self.__type = DataConnectionEventEnum.OPEN
        elif json["event"] == u"CLOSE":
            self.__type = DataConnectionEventEnum.CLOSE
        elif json["event"] == u"ERROR":
            self.__type = DataConnectionEventEnum.ERROR
            self.__error_message = json["error_message"]
        else:
            raise MyException("Unknown Event Type {}".format(json["event"]))

    def type(self):
        """
        Shows event type
        :return: event type
        :rtype: DataConnectionEventEnum
        """
        return self.__type

    def error_message(self):
        """
        error message
        :return: error messageq
        :rtype: unicode
        """
        return self.__error_message

    def json(self):
        json = {}
        if self.__type == DataConnectionEventEnum.OPEN:
            json["type"] = "OPEN"
        elif self.__type == DataConnectionEventEnum.CLOSE:
            json["type"] = "CLOSE"
        elif self.__type == DataConnectionEventEnum.ERROR:
            json["type"] = "ERROR"
            json["error_message"] = self.__error_message
        else:
            raise MyException("invalid event data")
        return json

    def __eq__(self, other):
        if not isinstance(other, DataConnectionEvent):
            return NotImplemented

        return self.json() == other.json()

    def __ne__(self, other):
        return not self.__eq__(other)


class Status:
    def __init__(self, json):
        """
        Shows status of DataConnection
        :param dict json:
        """

        self.remote_id = json["remote_id"]
        if not isinstance(self.remote_id, unicode):
            raise MyException("invalid parameter of remote_id in Status")

        self.buffersize = json["buffersize"]
        if not isinstance(self.buffersize, int):
            raise MyException("invalid parameter of buffersize in Status")

        self.label = json["label"]
        if not isinstance(self.label, unicode):
            raise MyException("invalid parameter of label in Status")

        self.metadata = json["metadata"]
        if not isinstance(self.metadata, unicode):
            raise MyException("invalid parameter of metadata in Status")

        self.open = json["open"]
        if not isinstance(self.open, bool):
            raise MyException("invalid parameter of open in Status")

        self.reliable = json["reliable"]
        if not isinstance(self.reliable, bool):
            raise MyException("invalid parameter of reliable in Status")

        self.serialization = json["serialization"]
        if not isinstance(self.serialization, unicode):
            raise MyException("invalid parameter of serialization in Status")

        self.type = json["type"]
        if not isinstance(self.type, unicode):
            raise MyException("invalid parameter of type in Status")

    def json(self):
        return {
            u"remote_id": self.remote_id,
            u"buffersize": self.buffersize,
            u"label": self.label,
            u"metadata": self.metadata,
            u"open": self.open,
            u"reliable": self.reliable,
            u"serialization": self.serialization,
            u"type": self.type,
        }

    def __eq__(self, other):
        if not isinstance(other, Status):
            return NotImplemented

        return (
            self.remote_id == other.remote_id
            and self.buffersize == other.buffersize
            and self.label == other.label
            and self.metadata == other.metadata
            and self.open == other.open
            and self.reliable == other.reliable
            and self.serialization == other.serialization
            and self.type == other.type
        )

    def __ne__(self, other):
        return not self.__eq__(other)


# FIXME: No Test
class DataEventItem:
    def __init__(self, type, json):
        self.__type = type
        self.__json = json

    def json(self):
        return self.__json

    def __eq__(self, other):
        if not isinstance(other, DataEventItem):
            return NotImplemented

        return self.json() == other.json()

    def __ne__(self, other):
        return not self.__eq__(other)
