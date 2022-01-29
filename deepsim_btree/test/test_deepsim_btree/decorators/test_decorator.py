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
from deepsim_btree.decorators.decorator import Decorator
from deepsim_btree.constants import Status
from deepsim_btree.leaves import Success, Failure, Running

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class DummyDecorator(Decorator):
    def __init__(self,
                 child: Behaviour,
                 name: Optional[str] = None,
                 update_side_effect: Any = None):
        super(DummyDecorator, self).__init__(name=name,
                                             child=child)
        self.mock = MagicMock()
        if update_side_effect is None:
            def default_update_side_effect():
                return Status.INVALID
            update_side_effect = default_update_side_effect
        self.mock.update.side_effect = update_side_effect

    def update(self) -> Status:
        return self.child.status


class DecoratorTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_init(self):
        child = Success()
        decorator = DummyDecorator(child=child)
        assert decorator.name == 'DummyDecorator'
        assert child == decorator.child
        assert child.parent == decorator

    def test_init_wrong_typed_child(self):
        child = 4.0
        with self.assertRaises(TypeError):
            _ = DummyDecorator(child=child)

    def test_setter(self):
        child = Success()
        decorator = DummyDecorator(child=child)
        assert decorator.child == child
        child2 = Failure()
        decorator.child = child2
        assert decorator.child == child2

    def test_setter_during_running(self):
        child = Running()
        decorator = DummyDecorator(child=child)
        decorator.tick()
        child2 = Success()
        with self.assertRaises(RuntimeError):
            decorator.child = child2

    def test_setter_wrong_typed_child(self):
        child = Success()
        decorator = DummyDecorator(child=child)
        with self.assertRaises(TypeError):
            decorator.child = 4.0

    def test_setter_child_with_parent(self):
        child = Success()
        decorator = DummyDecorator(child=child)
        random_parent = Success()
        child2 = Failure()
        child2.parent = random_parent
        with self.assertRaises(RuntimeError):
            decorator.child = child2

    def test_tick_success(self):
        child = Success()
        decorator = DummyDecorator(child=child)
        status = decorator.tick()
        assert status == Status.SUCCESS

        status = decorator.tick()
        assert status == Status.SUCCESS

    def test_tick_failure(self):
        child = Failure()
        decorator = DummyDecorator(child=child)

        status = decorator.tick()
        assert status == Status.FAILURE

        status = decorator.tick()
        assert status == Status.FAILURE

    def test_tick_running(self):
        child = Running()
        decorator = DummyDecorator(child=child)

        status = decorator.tick()
        assert status == Status.RUNNING

        status = decorator.tick()
        assert status == Status.RUNNING

    def test_stop(self):
        child = Running()
        decorator = DummyDecorator(child=child)

        status = decorator.tick()
        assert status == Status.RUNNING
        decorator.stop()
        assert decorator.status == Status.INVALID
        assert child.status == Status.INVALID

    def test_stop_success(self):
        child = Running()
        decorator = DummyDecorator(child=child)

        status = decorator.tick()
        assert status == Status.RUNNING
        decorator.stop(new_status=Status.SUCCESS)
        assert decorator.status == Status.SUCCESS
        assert child.status == Status.INVALID

    def test_stop_failure(self):
        child = Running()
        decorator = DummyDecorator(child=child)

        status = decorator.tick()
        assert status == Status.RUNNING
        decorator.stop(new_status=Status.FAILURE)
        assert decorator.status == Status.FAILURE
        assert child.status == Status.INVALID
