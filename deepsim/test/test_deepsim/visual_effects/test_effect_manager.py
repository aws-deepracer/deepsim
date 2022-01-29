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
from unittest.mock import patch, MagicMock, call
import inspect

from deepsim.visual_effects.effect_manager import EffectManager


myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class EffectManagerTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_add(self):
        effect_manager = EffectManager(is_singleton=False)
        effect = MagicMock()

        effect_manager.add(effect)
        assert effect in effect_manager._effects

    def test_remove(self):
        effect_manager = EffectManager(is_singleton=False)
        effect = MagicMock()

        effect_manager.add(effect)

        effect_manager.remove(effect)
        assert effect not in effect_manager._effects

    def test_remove_non_existing_randomizer(self):
        effect_manager = EffectManager(is_singleton=False)
        effect = MagicMock()

        with self.assertRaises(KeyError):
            effect_manager.remove(effect)

    def test_discard(self):
        effect_manager = EffectManager(is_singleton=False)
        effect = MagicMock()

        effect_manager.add(effect)

        effect_manager.discard(effect)
        assert effect not in effect_manager._effects

    def test_discard_non_existing_randomizer(self):
        effect_manager = EffectManager(is_singleton=False)
        effect = MagicMock()

        effect_manager.discard(effect)

    def test_on_update_tracker(self):
        effect_manager = EffectManager(is_singleton=False)
        effect = MagicMock()

        effect_manager.add(effect)

        delta_time_mock = MagicMock()
        sim_time_mock = MagicMock()
        effect_manager.on_update_tracker(delta_time=delta_time_mock,
                                         sim_time=sim_time_mock)
        effect.update.assert_called_once_with(delta_time_mock, sim_time_mock)
