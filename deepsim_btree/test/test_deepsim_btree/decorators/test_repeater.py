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

from deepsim_btree.decorators.repeater import Repeater
from deepsim_btree.constants import Status
from deepsim_btree.leaves import Success, Failure, Running

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class RepeaterTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_init(self):
        child = Success()
        repeater = Repeater(child=child)
        assert repeater.name == "Repeater"
        assert repeater.repeat == 0
        assert repeater.run_count == 0

    def test_setter(self):
        child = Success()
        repeater = Repeater(child=child)
        assert repeater.repeat == 0

        repeater.repeat = 2
        assert repeater.repeat == 2

    def test_setter_during_running(self):
        child = Running()
        repeater = Repeater(child=child)
        assert repeater.repeat == 0
        repeater.tick()
        assert repeater.status == Status.RUNNING
        with self.assertRaises(RuntimeError):
            repeater.repeat = 2

    def test_stop(self):
        child = Success()
        repeater = Repeater(child=child)
        assert repeater.run_count == 0
        repeater.tick()
        assert repeater.run_count == 1

        repeater.stop()
        repeater.tick()
        assert repeater.run_count == 1

    def test_update(self):
        child = Success()
        repeater = Repeater(child=child)
        repeater.repeat = 3
        status = repeater.tick()
        assert status == Status.RUNNING
        status = repeater.tick()
        assert status == Status.RUNNING
        status = repeater.tick()
        assert status == Status.RUNNING
        status = repeater.tick()
        assert status == Status.SUCCESS
