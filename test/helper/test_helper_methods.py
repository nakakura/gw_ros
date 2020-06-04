# -*- coding: utf-8 -*-
import pytest
import sys
from os import path
import multiprocessing
import Queue

sys.path.insert(
    0, path.dirname(path.dirname(path.dirname(path.abspath(__file__)))) + "/scripts",
)
from helper.methods import *


class TestHelperMethods:
    def setup_method(self, method):
        self.data = [
            {"name": "data", "redirect_params": {"ip_v4": "127.0.0.1", "port": 10000}},
            {"name": "data2", "redirect_params": {"ip_v6": "fe00::1", "port": 10001}},
        ]

    def teardown_method(self, method):
        del self.data

    def test_extract_config_found(self, mocker):
        assert extract_target_item("data", self.data) == (
            {"name": "data", "redirect_params": {"ip_v4": "127.0.0.1", "port": 10000}},
            [
                {
                    "name": "data2",
                    "redirect_params": {"ip_v6": "fe00::1", "port": 10001},
                }
            ],
        )

    def test_extract_config_not_found(self, mocker):
        assert extract_target_item("data3", self.data) == ({}, self.data)

    def test_extract_config_from_no_item_array(self, mocker):
        assert extract_target_item("data3", []) == ({}, [],)
