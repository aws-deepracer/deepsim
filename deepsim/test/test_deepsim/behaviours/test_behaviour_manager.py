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
from typing import Any, Callable
from unittest import TestCase
from unittest.mock import MagicMock
import inspect

from deepsim.behaviours.behaviour_manager import BehaviourManager


myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class BehaviourManagerTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_add(self):
        behaviour_manager = BehaviourManager(is_singleton=False)
        behaviour = MagicMock()
        name = myself()
        tag = myself() + '_tag'
        behaviour.name = name
        behaviour.tags = {tag}
        behaviour_manager.add(behaviour)
        assert name in behaviour_manager._behaviour_map
        assert tag in behaviour_manager._tag_map
        assert behaviour in behaviour_manager._tag_map[tag]

    def test_get(self):
        behaviour_manager = BehaviourManager(is_singleton=False)
        behaviour = MagicMock()
        name = myself()
        tag = myself() + '_tag'
        behaviour.name = name
        behaviour.tags = {tag}
        behaviour_manager.add(behaviour)

        assert behaviour == behaviour_manager.get(name)

    def test_get_behaviour_missing(self):
        behaviour_manager = BehaviourManager(is_singleton=False)
        behaviour = MagicMock()

        assert behaviour == behaviour_manager.get("test_get2", behaviour)

    def test_find_by_tag(self):
        behaviour_manager = BehaviourManager(is_singleton=False)
        behaviour = MagicMock()
        name = myself()
        tag = myself() + '_tag'
        behaviour.name = name
        behaviour.tags = {tag}
        behaviour_manager.add(behaviour)

        assert behaviour in behaviour_manager.find_by_tag(tag)

    def test_remove_with_obj(self):
        behaviour_manager = BehaviourManager(is_singleton=False)
        behaviour = MagicMock()
        name = myself()
        tag = myself() + '_tag'
        behaviour.name = name
        behaviour.tags = {tag}
        behaviour_manager.add(behaviour)

        behaviour_manager.remove(behaviour)

        assert behaviour not in behaviour_manager._behaviour_map
        assert behaviour not in behaviour_manager._tag_map[tag]

    def test_remove_with_name(self):
        behaviour_manager = BehaviourManager(is_singleton=False)
        behaviour = MagicMock()
        name = myself()
        tag = myself() + '_tag'
        behaviour.name = name
        behaviour.tags = {tag}
        behaviour_manager.add(behaviour)

        behaviour_manager.remove(name)

        assert behaviour not in behaviour_manager._behaviour_map
        assert behaviour not in behaviour_manager._tag_map[tag]

    def test_remove_non_existing_behaviour(self):
        behaviour_manager = BehaviourManager(is_singleton=False)
        behaviour = MagicMock()
        name = myself()
        tag = myself() + '_tag'
        behaviour.name = name
        behaviour.tags = {tag}
        with self.assertRaises(KeyError):
            behaviour_manager.remove(name)

    def test_discard_with_obj(self):
        behaviour_manager = BehaviourManager(is_singleton=False)
        behaviour = MagicMock()
        name = myself()
        tag = myself() + '_tag'
        behaviour.name = name
        behaviour.tags = {tag}
        behaviour_manager.add(behaviour)

        behaviour_manager.discard(behaviour)

        assert behaviour not in behaviour_manager._behaviour_map
        assert behaviour not in behaviour_manager._tag_map[tag]

    def test_discard_with_name(self):
        behaviour_manager = BehaviourManager(is_singleton=False)
        behaviour = MagicMock()
        name = myself()
        tag = myself() + '_tag'
        behaviour.name = name
        behaviour.tags = {tag}
        behaviour_manager.add(behaviour)

        behaviour_manager.discard(name)

        assert behaviour not in behaviour_manager._behaviour_map
        assert behaviour not in behaviour_manager._tag_map[tag]

    def test_discard_non_existing_behaviour(self):
        behaviour_manager = BehaviourManager(is_singleton=False)
        behaviour = MagicMock()
        name = myself()
        tag = myself() + '_tag'
        behaviour.name = name
        behaviour.tags = {tag}
        behaviour_manager.discard(name)

    def test_update(self):
        behaviour_manager = BehaviourManager(is_singleton=False)
        behaviour = MagicMock()
        name = myself()
        tag = myself() + '_tag'
        behaviour.name = name
        behaviour.tags = {tag}
        behaviour_manager.add(behaviour)

        behaviour_manager.update()
        behaviour.update.assert_called_once()

    def test_update_with_tag(self):
        behaviour_manager = BehaviourManager(is_singleton=False)
        behaviour = MagicMock()
        name = myself()
        tag = myself() + '_tag'
        behaviour.name = name
        behaviour.tags = {tag}
        behaviour_manager.add(behaviour)

        behaviour_manager.update(tag=tag)
        behaviour.update.assert_called_once()

    def test_update_with_wrong_tag(self):
        behaviour_manager = BehaviourManager(is_singleton=False)
        behaviour = MagicMock()
        name = myself()
        tag = myself() + '_tag'
        behaviour.name = name
        behaviour.tags = {tag}
        behaviour_manager.add(behaviour)

        behaviour_manager.update(tag="wrong_tag")
        behaviour.update.assert_not_called()

    def test_reset(self):
        behaviour_manager = BehaviourManager(is_singleton=False)
        behaviour = MagicMock()
        name = myself()
        tag = myself() + '_tag'
        behaviour.name = name
        behaviour.tags = {tag}
        behaviour_manager.add(behaviour)

        behaviour_manager.reset()
        behaviour.reset.assert_called_once()

    def test_reset_with_tag(self):
        behaviour_manager = BehaviourManager(is_singleton=False)
        behaviour = MagicMock()
        name = myself()
        tag = myself() + '_tag'
        behaviour.name = name
        behaviour.tags = {tag}
        behaviour_manager.add(behaviour)

        behaviour_manager.reset(tag=tag)
        behaviour.reset.assert_called_once()

    def test_reset_with_wrong_tag(self):
        behaviour_manager = BehaviourManager(is_singleton=False)
        behaviour = MagicMock()
        name = myself()
        tag = myself() + '_tag'
        behaviour.name = name
        behaviour.tags = {tag}
        behaviour_manager.add(behaviour)

        behaviour_manager.reset(tag="wrong_tag")
        behaviour.reset.assert_not_called()

    def test_on_update_tracker(self):
        behaviour_manager = BehaviourManager(is_singleton=False)
        behaviour = MagicMock()
        name = myself()
        tag = myself() + '_tag'
        behaviour.name = name
        behaviour.tags = {tag}
        behaviour_manager.add(behaviour)

        delta_time = MagicMock()
        sim_time = MagicMock()
        behaviour_manager.on_update_tracker(delta_time, sim_time)
        behaviour.fixed_update.assert_called_once_with(delta_time=delta_time,
                                                      sim_time=sim_time)
