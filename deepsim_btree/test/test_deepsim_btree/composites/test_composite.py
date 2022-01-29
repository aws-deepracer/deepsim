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

from deepsim_btree.composites.composite import Composite
from deepsim_btree.behaviour import Behaviour
from deepsim_btree.constants import Status
from deepsim_btree.leaves import Success, Failure, Running

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class RunningComposite(Composite):
    def __init__(self,
                 children: Optional[Iterable[Behaviour]] = None,
                 name: Optional[str] = None):
        super(RunningComposite, self).__init__(name=name,
                                               children=children)

    def update(self) -> Status:
        return Status.RUNNING


class CompositeTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_init(self):
        composite = Composite()
        assert composite.name == "Composite"
        assert composite.children == []

        children = [Success(), Failure(), Success()]
        composite = Composite(children=children)
        assert composite.children == children

    def test_stop(self):
        running = Running()
        running.tick()
        children = [Success(), running, Success()]

        composite = Composite(children=children)
        assert composite.children == children
        composite.stop()
        for child in composite.children:
            assert child.status != Status.RUNNING
        assert composite.status == Status.INVALID

    def test_add_child(self):
        children = [Success(), Failure(), Success()]
        composite = Composite(children=children)
        child = Running()

        child_id = composite.add_child(child=child)
        assert child in composite.children
        assert child_id == child.id
        assert child.parent == composite

    def test_add_child_wrong_typed_child(self):
        children = [Success(), Failure(), Success()]
        composite = Composite(children=children)
        with self.assertRaises(TypeError):
            composite.add_child(42.0)

    def test_add_child_with_parent(self):
        children = [Success(), Failure(), Success()]
        composite = Composite(children=children)

        parent = Success()
        child = Running()
        child.parent = parent

        with self.assertRaises(RuntimeError):
            composite.add_child(child)

    def test_add_child_during_running(self):
        children = [Success(), Failure(), Success()]
        composite = RunningComposite(children=children)
        composite.tick()

        child = Running()
        with self.assertRaises(RuntimeError):
            composite.add_child(child)

    def test_add_children(self):
        composite = Composite()

        children = [Success(), Failure(), Success()]

        obj = composite.add_children(children=children)
        assert obj == composite
        assert children == composite.children

    def test_add_children_wrong_typed_child(self):
        composite = Composite()
        children = [42.0, 32.0, 22.2]

        with self.assertRaises(TypeError):
            composite.add_children(children=children)

    def test_add_children_with_parent(self):
        composite = Composite()

        parent = Success()
        child_with_parent = Running()
        child_with_parent.parent = parent
        children = [Success(), child_with_parent, Success()]

        with self.assertRaises(RuntimeError):
            composite.add_children(children=children)

    def test_add_children_during_running(self):

        composite = RunningComposite()
        composite.tick()

        children = [Success(), Failure(), Success()]
        with self.assertRaises(RuntimeError):
            composite.add_children(children=children)

    def test_remove_child(self):
        children = [Success(), Failure(), Success()]
        composite = Composite(children=children)
        child = Running()

        child_id = composite.add_child(child=child)
        assert child.parent == composite
        assert child in composite.children
        assert child_id == child.id
        removed_child_idx = composite.remove_child(child=child)
        assert child not in composite.children
        assert removed_child_idx == 3
        assert child.parent is None

    def test_remove_running_child(self):
        children = [Success(), Failure(), Success()]
        composite = Composite(children=children)
        child = Running()
        child.tick()

        child_id = composite.add_child(child=child)
        assert child in composite.children
        assert child_id == child.id
        removed_child_idx = composite.remove_child(child=child)
        assert child not in composite.children
        assert removed_child_idx == 3
        assert child.status == Status.INVALID

    def test_remove_child_during_running(self):
        children = [Success(), Failure(), Success()]
        composite = RunningComposite(children=children)
        child = Running()

        child_id = composite.add_child(child=child)
        assert child in composite.children
        assert child_id == child.id

        composite.tick()
        with self.assertRaises(RuntimeError):
            composite.remove_child(child=child)

    def test_remove_all_children(self):
        children = [Success(), Failure(), Success()]
        composite = Composite(children=children)

        composite.remove_all_children()
        assert composite.children == []

    def test_remove_all_children_with_running_child(self):
        children = [Success(), Failure(), Success()]
        composite = Composite(children=children)
        child = Running()
        child.tick()

        child_id = composite.add_child(child=child)
        assert child in composite.children
        assert child_id == child.id
        composite.remove_all_children()
        assert composite.children == []
        assert child.status == Status.INVALID

    def test_remove_all_children_during_running(self):
        children = [Success(), Failure(), Success()]
        composite = RunningComposite(children=children)
        composite.tick()

        with self.assertRaises(RuntimeError):
            composite.remove_all_children()

    def test_replace_child(self):
        children = [Success(), Failure(), Success()]
        composite = Composite(children=children)
        child = Running()

        child_id = composite.add_child(child=child)
        assert child.parent == composite
        assert child in composite.children
        assert child_id == child.id

        replacement = Success()
        replacement_idx = composite.replace_child(child=child,
                                                  replacement=replacement)
        assert child not in composite.children
        assert child.parent is None
        assert replacement in composite.children
        assert replacement.parent == composite
        assert replacement_idx == composite.children.index(replacement)

    def test_replace_child_during_running(self):
        children = [Success(), Failure(), Success()]
        composite = RunningComposite(children=children)

        child = Running()

        child_id = composite.add_child(child=child)
        assert child.parent == composite
        assert child in composite.children
        assert child_id == child.id

        composite.tick()

        replacement = Success()
        with self.assertRaises(RuntimeError):
            composite.replace_child(child=child,
                                    replacement=replacement)

    def test_remove_child_by_id(self):
        children = [Success(), Failure(), Success()]
        composite = Composite(children=children)
        child = Running()

        child_id = composite.add_child(child=child)
        assert child.parent == composite
        assert child in composite.children
        assert child_id == child.id

        composite.remove_child_by_id(child_id=child_id)

        assert child not in composite.children
        assert child.parent is None

    def test_remove_child_by_id_not_found(self):
        children = [Success(), Failure(), Success()]
        composite = Composite(children=children)

        child = Running()
        with self.assertRaises(IndexError):
            composite.remove_child_by_id(child_id=child.id)

    def test_remove_child_by_id_during_running(self):
        children = [Success(), Failure(), Success()]
        composite = RunningComposite(children=children)
        child = Running()

        child_id = composite.add_child(child=child)
        assert child in composite.children
        assert child_id == child.id

        composite.tick()
        with self.assertRaises(RuntimeError):
            composite.remove_child_by_id(child_id=child_id)

    def test_prepend_child(self):
        children = [Success(), Failure(), Success()]
        composite = Composite(children=children)
        child = Running()

        child_id = composite.prepend_child(child=child)
        assert child == composite.children[0]
        assert child_id == child.id
        assert child.parent == composite

    def test_prepend_child_wrong_typed_child(self):
        children = [Success(), Failure(), Success()]
        composite = Composite(children=children)
        with self.assertRaises(TypeError):
            composite.prepend_child(42.0)

    def test_prepend_child_with_parent(self):
        children = [Success(), Failure(), Success()]
        composite = Composite(children=children)

        parent = Success()
        child = Running()
        child.parent = parent

        with self.assertRaises(RuntimeError):
            composite.prepend_child(child)

    def test_prepend_child_during_running(self):
        children = [Success(), Failure(), Success()]
        composite = RunningComposite(children=children)
        composite.tick()

        child = Running()
        with self.assertRaises(RuntimeError):
            composite.prepend_child(child)

    def test_insert_child(self):
        children = [Success(), Failure(), Success()]
        composite = Composite(children=children)
        child = Running()

        child_id = composite.insert_child(child, 2)
        assert child == composite.children[2]
        assert child_id == child.id
        assert child.parent == composite

    def test_insert_child_wrong_typed_child(self):
        children = [Success(), Failure(), Success()]
        composite = Composite(children=children)
        with self.assertRaises(TypeError):
            composite.insert_child(42.0, 2)

    def test_insert_child_with_parent(self):
        children = [Success(), Failure(), Success()]
        composite = Composite(children=children)

        parent = Success()
        child = Running()
        child.parent = parent

        with self.assertRaises(RuntimeError):
            composite.insert_child(child, 2)

    def test_insert_child_during_running(self):
        children = [Success(), Failure(), Success()]
        composite = RunningComposite(children=children)
        composite.tick()

        child = Running()
        with self.assertRaises(RuntimeError):
            composite.insert_child(child, 2)
