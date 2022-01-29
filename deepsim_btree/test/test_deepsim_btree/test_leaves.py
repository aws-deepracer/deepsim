#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################
from typing import Any, Callable, Optional, Iterable
from unittest import mock, TestCase
from unittest.mock import patch, MagicMock, call
import inspect

from deepsim_btree.leaves import Success, Failure, Running
from deepsim_btree.constants import Status

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class BehaviourTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_success(self):
        behavior = Success()
        assert behavior.name == "Success"
        status = behavior.tick()
        assert status == Status.SUCCESS
        status = behavior.tick()
        assert status == Status.SUCCESS

    def test_failure(self):
        behavior = Failure()
        assert behavior.name == "Failure"
        status = behavior.tick()
        assert status == Status.FAILURE
        status = behavior.tick()
        assert status == Status.FAILURE

    def test_running(self):
        behavior = Running()
        assert behavior.name == "Running"
        status = behavior.tick()
        assert status == Status.RUNNING
        status = behavior.tick()
        assert status == Status.RUNNING
