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

from deepsim_btree.behaviour import Behaviour
from deepsim_btree.constants import Status

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class DummyBehaviour(Behaviour):
    def __init__(self,
                 name: Optional[str] = None,
                 update_side_effect: Any = None):
        super(DummyBehaviour, self).__init__(name=name)
        self.mock = MagicMock()
        if update_side_effect is None:
            def default_update_side_effect():
                return Status.INVALID
            update_side_effect = default_update_side_effect
        self.mock.update.side_effect = update_side_effect

    def initialize(self) -> None:
        self.mock.initialize()

    def terminate(self, new_status: Status) -> None:
        self.mock.terminate(new_status=new_status)

    def update(self) -> Status:
        return self.mock.update()


class BehaviourTest(TestCase):
    def setUp(self) -> None:
        self.call_count = 0

    def test_init(self) -> None:
        behaviour = DummyBehaviour()
        assert behaviour.name == "DummyBehaviour"
        assert behaviour.feedback_message == ""
        assert behaviour.status == Status.INVALID
        assert behaviour.parent is None

        name = myself()
        behaviour = DummyBehaviour(name=name)
        assert behaviour.name == name

    def test_initialize_wrong_name_type(self) -> None:
        with self.assertRaises(TypeError):
            _ = DummyBehaviour(name=10.3)

    def test_setters(self) -> None:
        behaviour = DummyBehaviour()
        assert behaviour.name == "DummyBehaviour"
        assert behaviour.feedback_message == ""
        assert behaviour.status == Status.INVALID
        assert behaviour.parent is None

        parent = DummyBehaviour()
        behaviour.parent = parent
        assert behaviour.parent == parent

    def test_tick(self):
        behaviour = DummyBehaviour()
        status = behaviour.tick()
        behaviour.mock.initialize.assert_called_once()
        assert status == Status.INVALID
        behaviour.mock.update.assert_called_once()
        behaviour.mock.terminate.assert_called_once_with(new_status=Status.INVALID)
        assert behaviour.status == Status.INVALID

    def test_tick_long_success(self):
        def update():
            self.call_count += 1
            if self.call_count < 3:
                return Status.RUNNING
            else:
                return Status.SUCCESS

        behaviour = DummyBehaviour(update_side_effect=update)
        status = behaviour.tick()
        behaviour.mock.initialize.assert_called_once()
        behaviour.mock.update.assert_called_once()
        assert status == Status.RUNNING

        status = behaviour.tick()
        assert status == Status.RUNNING

        status = behaviour.tick()
        assert status == Status.SUCCESS
        assert behaviour.status == Status.SUCCESS
        behaviour.mock.terminate.assert_called_once_with(new_status=Status.SUCCESS)

    def test_tick_long_failure(self):
        def update():
            self.call_count += 1
            if self.call_count < 3:
                return Status.RUNNING
            else:
                return Status.FAILURE

        behaviour = DummyBehaviour(update_side_effect=update)
        status = behaviour.tick()
        behaviour.mock.initialize.assert_called_once()
        behaviour.mock.update.assert_called_once()
        assert status == Status.RUNNING

        status = behaviour.tick()
        assert status == Status.RUNNING

        status = behaviour.tick()
        assert status == Status.FAILURE
        assert behaviour.status == Status.FAILURE
        behaviour.mock.terminate.assert_called_once_with(new_status=Status.FAILURE)

    def test_tick_stop(self):
        def update():
            self.call_count += 1
            if self.call_count < 3:
                return Status.RUNNING
            else:
                return Status.FAILURE

        behaviour = DummyBehaviour(update_side_effect=update)
        status = behaviour.tick()
        behaviour.mock.initialize.assert_called_once()
        behaviour.mock.update.assert_called_once()
        assert status == Status.RUNNING

        behaviour.stop()
        behaviour.mock.terminate.assert_called_once_with(new_status=Status.INVALID)
        assert behaviour.status == Status.INVALID

    def test_tick_stop_success(self):
        def update():
            self.call_count += 1
            if self.call_count < 3:
                return Status.RUNNING
            else:
                return Status.FAILURE

        behaviour = DummyBehaviour(update_side_effect=update)
        status = behaviour.tick()
        behaviour.mock.initialize.assert_called_once()
        behaviour.mock.update.assert_called_once()
        assert status == Status.RUNNING

        behaviour.stop(new_status=Status.SUCCESS)
        behaviour.mock.terminate.assert_called_once_with(new_status=Status.SUCCESS)
        assert behaviour.status == Status.SUCCESS

    def test_tick_stop_failure(self):
        def update():
            self.call_count += 1
            if self.call_count < 3:
                return Status.RUNNING
            else:
                return Status.FAILURE

        behaviour = DummyBehaviour(update_side_effect=update)
        status = behaviour.tick()
        behaviour.mock.initialize.assert_called_once()
        behaviour.mock.update.assert_called_once()
        assert status == Status.RUNNING

        behaviour.stop(new_status=Status.FAILURE)
        behaviour.mock.terminate.assert_called_once_with(new_status=Status.FAILURE)
        assert behaviour.status == Status.FAILURE





