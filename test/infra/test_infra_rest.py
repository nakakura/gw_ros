#!/usr/bin/env python
# -*- coding: utf-8 -*-
import logging
import sys
import unittest
from os import path
from mock import *

sys.path.append(path.dirname(path.dirname(path.dirname(path.abspath(__file__)))))
sys.path.append(
    path.dirname(path.dirname(path.dirname(path.abspath(__file__)))) + "/scripts"
)
from infra.rest import Rest
from error import MyException

PKG = "skyway"


class MockResponse:
    def __init__(self, url, json_data, status_code):
        self.json_data = json_data
        self.status_code = status_code
        self.url = url

    def json(self):
        return self.json_data


class TestRest(unittest.TestCase):
    # get success
    def test_get_success(self):
        with patch(
            "requests.get",
            return_value=MockResponse("dummy_url", {"key1": "value1"}, 200),
        ) as mock_get:
            rest = Rest("domain")
            response = rest.get("url_200", 200)
            self.assertTrue(mock_get.called)
            self.assertEqual(mock_get.call_args[0][0], "domain/url_200")
            self.assertEqual(response, {"key1": "value1"})

    # get success_but_status_code_wrong
    def test_get_success_with_wrong_code(self):
        with patch(
            "requests.get",
            return_value=MockResponse("dummy_url", {"key1": "value1"}, 201),
        ):
            try:
                rest = Rest("domain")
                _response = rest.get("url_201", 200)
            except Exception as e:
                self.assertEqual(
                    e,
                    MyException(
                        {
                            "url": "dummy_url",
                            "error": "Unexpected Status Code",
                            "status": 201,
                        }
                    ),
                )

    # -------------------- POST --------------------
    # success case
    def test_post_success(self):
        with patch(
            "requests.post",
            return_value=MockResponse("dummy_url", {"key1": "value1"}, 201),
        ) as mock_post:
            rest = Rest("domain")
            response = rest.post("url_201", {"param": "valid"}, 201)
            self.assertEqual(response, {"key1": "value1"})

    # return 400
    def test_post_400(self):
        with patch(
            "requests.post",
            return_value=MockResponse("dummy_url", {"key1": "value1"}, 400),
        ) as mock_post:
            try:
                rest = Rest("domain")
                _response = rest.post("url_201", {"param": "valid"}, 201)
            except Exception as e:
                self.assertEqual(
                    e,
                    MyException(
                        {"url": "dummy_url", "error": "400 Bad Request", "status": 400}
                    ),
                )

    # return 403
    def test_post_403(self):
        with patch(
            "requests.post",
            return_value=MockResponse("dummy_url", {"key1": "value1"}, 403),
        ):
            try:
                rest = Rest("domain")
                _response = rest.post("url_201", {"param": "valid"}, 201)
            except Exception as e:
                self.assertEqual(
                    e,
                    MyException(
                        {"url": "dummy_url", "error": "403 Forbidden", "status": 403}
                    ),
                )

    # return 404
    def test_post_404(self):
        with patch(
            "requests.post",
            return_value=MockResponse("dummy_url", {"key1": "value1"}, 404),
        ):
            try:
                rest = Rest("domain")
                _response = rest.post("url_201", {"param": "valid"}, 201)
            except Exception as e:
                self.assertEqual(
                    e,
                    MyException(
                        {"url": "dummy_url", "error": "404 Not Found", "status": 404}
                    ),
                )

    # return 405
    def test_post_405(self):
        with patch(
            "requests.post",
            return_value=MockResponse("dummy_url", {"key1": "value1"}, 405),
        ):
            try:
                rest = Rest("domain")
                _response = rest.post("url_201", {"param": "valid"}, 201)
            except Exception as e:
                self.assertEqual(
                    e,
                    MyException(
                        {
                            "url": "dummy_url",
                            "error": "405 Method Not Allowed",
                            "status": 405,
                        }
                    ),
                )

    # return 406
    def test_post_406(self):
        with patch(
            "requests.post",
            return_value=MockResponse("dummy_url", {"key1": "value1"}, 406),
        ):
            try:
                rest = Rest("domain")
                _response = rest.post("url_201", {"param": "valid"}, 201)
            except Exception as e:
                self.assertEqual(
                    e,
                    MyException(
                        {
                            "url": "dummy_url",
                            "error": "406 Not Acceptable",
                            "status": 406,
                        }
                    ),
                )

    # return 408
    def test_post_408(self):
        with patch(
            "requests.post",
            return_value=MockResponse("dummy_url", {"key1": "value1"}, 408),
        ):
            try:
                rest = Rest("domain")
                _response = rest.post("url_201", {"param": "valid"}, 201)
            except Exception as e:
                self.assertEqual(
                    e,
                    MyException(
                        {
                            "url": "dummy_url",
                            "error": "408 Request Timeout",
                            "status": 408,
                        }
                    ),
                )

    # -------------------- DELETE --------------------
    # delete success
    def test_delete_success(self):
        with patch(
            "requests.delete",
            return_value=MockResponse("dummy_url", {"key1": "value1"}, 204),
        ) as mock_delete:
            rest = Rest("domain")
            response = rest.delete("url_204", 204)
            self.assertEqual(response, {"key1": "value1"})
            self.assertTrue(mock_delete.called)
            self.assertEqual(mock_delete.call_args[0][0], "domain/url_204")

    # -------------------- PUT --------------------
    # put success
    def test_put_success(self):
        with patch(
            "requests.put",
            return_value=MockResponse("dummy_url", {"key1": "value1"}, 201),
        ) as mock_put:
            rest = Rest("domain")
            response = rest.put("url_201", {"param": "valid"}, 201)
            self.assertEqual(response, {"key1": "value1"})
            self.assertTrue(mock_put.called)
            self.assertEqual(mock_put.call_args[0][0], "domain/url_201")


if __name__ == "__main__":
    import rostest

    logging.basicConfig(stream=sys.stderr, level=logging.DEBUG)
    rostest.rosrun(PKG, "data_redirect", TestRest)
