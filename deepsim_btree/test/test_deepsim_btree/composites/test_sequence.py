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

from deepsim_btree.composites.sequence import Sequence
from deepsim_btree.constants import Status
from deepsim_btree.leaves import Success, Failure, Running

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class SequenceTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_init(self):
        sequence = Sequence()
        assert sequence.current_child is None

    def test_current_child(self):
        running_child = Running()
        children = [Success(), running_child, Success()]
        sequence = Sequence(children=children)
        status = sequence.tick()
        assert status == Status.RUNNING
        assert sequence.current_child == running_child

    def test_tick_success(self):
        children = [Success(), Success(), Success()]
        sequence = Sequence(children=children)
        status = sequence.tick()
        assert status == Status.SUCCESS

    def test_tick_failure(self):
        children = [Success(), Success(), Failure(), Success()]
        sequence = Sequence(children=children)
        status = sequence.tick()
        assert status == Status.FAILURE

    def test_tick_empty(self):
        sequence = Sequence()
        status = sequence.tick()
        assert status == Status.SUCCESS
