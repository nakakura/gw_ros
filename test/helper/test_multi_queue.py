# -*- coding: utf-8 -*-
import pytest
import sys
from os import path
import multiprocessing
import Queue

sys.path.insert(
    0, path.dirname(path.dirname(path.dirname(path.abspath(__file__)))) + "/scripts",
)
from helper.multi_queue import MultiQueue


class TestMultiQueue:
    def setup_method(self, method):
        self.queue1 = multiprocessing.Queue()
        self.queue2 = multiprocessing.Queue()
        self.multi_queue = MultiQueue(self.queue1, self.queue2)

    def teardown_method(self, method):
        del self.multi_queue
        del self.queue1
        del self.queue2

    def test_generate(self, mocker):
        self.queue2.put("data2")
        self.queue2.put("data4")
        self.queue1.put("data1")
        self.queue1.put("data3")
        generator = self.multi_queue.generate()
        assert generator.next() == "data1"
        assert generator.next() == "data2"
        assert generator.next() == "data3"
        assert generator.next() == "data4"

    def test_get(self):
        self.queue2.put("data2")
        self.queue2.put("data4")
        self.queue1.put("data1")
        self.queue1.put("data3")
        assert self.multi_queue.get(0.1) == "data1"
        assert self.multi_queue.get(0.1) == "data3"
        assert self.multi_queue.get(0.1) == "data2"
        assert self.multi_queue.get(0.1) == "data4"
        with pytest.raises(Queue.Empty):
            self.multi_queue.get(0.1)
