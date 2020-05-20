# -*- coding: utf-8 -*-
from abc import ABCMeta, abstractmethod

from model import DataSocket


class IDataApi:
    __metaclass__ = ABCMeta

    @abstractmethod
    def open_data_socket_request(self):
        # type: () -> DataSocket
        pass
