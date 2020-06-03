# -*- coding: utf-8 -*-
import pytest
import sys
from os import path
import multiprocessing
import Queue

sys.path.insert(
    0, path.dirname(path.dirname(path.dirname(path.abspath(__file__)))) + "/scripts",
)
from helper.queue_generator import QueueGenerator


class TestQueueGenerator:
    def setup_method(self, method):
        self.queue = multiprocessing.Queue()
        self.generator = QueueGenerator(self.queue)

    def teardown_method(self, method):
        del self.generator
        del self.queue

    def test_multi_queue(self, mocker):
        self.queue.put("data1")
        self.queue.put("data2")
        self.queue.put("data3")
        generator = self.generator.generate()
        assert generator.next() == "data1"
        assert generator.next() == "data2"
        assert generator.next() == "data3"
