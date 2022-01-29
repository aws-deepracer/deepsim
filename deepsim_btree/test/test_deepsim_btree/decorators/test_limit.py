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

from deepsim_btree.decorators.limit import Limit
from deepsim_btree.constants import Status
from deepsim_btree.leaves import Success, Failure, Running

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class LimitTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_init(self):
        child = Success()
        limit = Limit(child=child)
        assert limit.name == "Limit"
        assert limit.tick_limit is None
        assert limit.tick_count == 0

    def test_setter(self):
        child = Success()
        limit = Limit(child=child)
        assert limit.tick_count == 0

        limit.tick_limit = 10
        assert limit.tick_limit == 10

    def test_setter_during_running(self):
        child = Running()
        limit = Limit(child=child)
        assert limit.tick_count == 0
        limit.tick()

        assert limit.tick_count == 1
        with self.assertRaises(RuntimeError):
            limit.tick_limit = 10

    def test_stop(self):
        child = Running()
        limit = Limit(child=child)
        assert limit.tick_count == 0
        limit.tick()
        assert limit.tick_count == 1

        limit.stop()
        limit.tick()
        assert limit.tick_count == 1

    def test_update(self):
        child = Running()
        limit = Limit(child=child)
        status = limit.tick()
        assert status == Status.RUNNING

        child = Failure()
        limit = Limit(child=child)
        status = limit.tick()
        assert status == Status.FAILURE

        child = Success()
        limit = Limit(child=child)
        status = limit.tick()
        assert status == Status.SUCCESS

    def test_update_exceeds_tick_limit(self):
        child = Running()
        limit = Limit(child=child,
                      tick_limit=2)
        status = limit.tick()
        assert status == Status.RUNNING
        status = limit.tick()
        assert status == Status.RUNNING
        status = limit.tick()
        assert status == Status.FAILURE
