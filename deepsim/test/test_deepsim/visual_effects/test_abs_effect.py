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

from deepsim.visual_effects.abs_effect import AbstractEffect
from deepsim.exception import DeepSimCallbackError

from rosgraph_msgs.msg import Clock


class DummyEffect(AbstractEffect):
    def __init__(self):
        self.mock = MagicMock()
        super(DummyEffect, self).__init__()

    def _lazy_init(self) -> None:
        self.mock.lazy_init()

    def on_attach_effect(self) -> None:
        self.mock.on_attach_effect()

    def on_detach_effect(self) -> None:
        self.mock.on_detach_effect()

    def on_update_effect(self, delta_time: float, sim_time: Clock) -> None:
        self.mock.on_update_effect(delta_time, sim_time)


class AbstractEffectTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_register(self):
        effect = DummyEffect()
        observer_mock = MagicMock()

        effect.register(observer_mock)
        assert observer_mock in effect._observers

    def test_unregister(self):
        effect = DummyEffect()
        observer_mock = MagicMock()

        effect.register(observer_mock)
        assert observer_mock in effect._observers

        effect.unregister(observer_mock)
        assert observer_mock not in effect._observers

    def test_attach(self):
        effect = DummyEffect()
        observer_mock = MagicMock()

        effect.register(observer_mock)
        assert observer_mock in effect._observers
        with patch("deepsim.visual_effects.abs_effect.EffectManager") as effect_manager_mock:

            effect.attach()
            effect.mock.on_attach_effect.assert_called_once()
            observer_mock.on_attach_effect.assert_called_once_with(effect)
            effect_manager_mock.get_instance.return_value.add.assert_called_once_with(effect)

    def test_ignore_attach_twice(self):
        effect = DummyEffect()
        observer_mock = MagicMock()

        effect.register(observer_mock)
        assert observer_mock in effect._observers
        with patch("deepsim.visual_effects.abs_effect.EffectManager") as effect_manager_mock:
            effect.attach()
            effect.attach()
            effect.mock.on_attach_effect.assert_called_once()
            observer_mock.on_attach_effect.assert_called_once_with(effect)
            effect_manager_mock.get_instance.return_value.add.assert_called_once_with(effect)

    def test_detach(self):
        effect = DummyEffect()
        observer_mock = MagicMock()

        effect.register(observer_mock)
        assert observer_mock in effect._observers
        with patch("deepsim.visual_effects.abs_effect.EffectManager") as effect_manager_mock:
            effect.attach()
            effect.detach()

            effect.mock.on_attach_effect.assert_called_once()
            observer_mock.on_attach_effect.assert_called_once_with(effect)
            effect_manager_mock.get_instance.return_value.add.assert_called_once_with(effect)

            effect.mock.on_detach_effect.assert_called_once()
            observer_mock.on_detach_effect.assert_called_once_with(effect)
            effect_manager_mock.get_instance.return_value.discard.assert_called_once_with(effect)

    def test_ignore_detach_without_attach(self):
        effect = DummyEffect()
        observer_mock = MagicMock()

        effect.register(observer_mock)
        assert observer_mock in effect._observers
        with patch("deepsim.visual_effects.abs_effect.EffectManager") as effect_manager_mock:
            effect.detach()

            effect.mock.on_detach_effect.assert_not_called()
            observer_mock.on_detach_effect.assert_not_called()
            effect_manager_mock.get_instance.return_value.discard.assert_not_called()

    def test_ignore_detach_twice(self):
        effect = DummyEffect()
        observer_mock = MagicMock()

        effect.register(observer_mock)
        assert observer_mock in effect._observers
        with patch("deepsim.visual_effects.abs_effect.EffectManager") as effect_manager_mock:
            effect.attach()
            effect.detach()
            effect.detach()

            effect.mock.on_attach_effect.assert_called_once()
            observer_mock.on_attach_effect.assert_called_once_with(effect)
            effect_manager_mock.get_instance.return_value.add.assert_called_once_with(effect)

            effect.mock.on_detach_effect.assert_called_once()
            observer_mock.on_detach_effect.assert_called_once_with(effect)
            effect_manager_mock.get_instance.return_value.discard.assert_called_once_with(effect)

    def test_attach_callback_failed(self):
        effect = DummyEffect()
        observer_mock = MagicMock()
        observer_mock.on_attach_effect.side_effect = Exception()

        effect.register(observer_mock)
        assert observer_mock in effect._observers
        with patch("deepsim.visual_effects.abs_effect.EffectManager") as effect_manager_mock:
            with self.assertRaises(DeepSimCallbackError):
                effect.attach()
            effect_manager_mock.get_instance.return_value.add.assert_called_once_with(effect)
            effect.mock.on_attach_effect.assert_called_once()
            observer_mock.on_attach_effect.assert_called_once_with(effect)

    def test_detach_callback_failed(self):
        effect = DummyEffect()
        observer_mock = MagicMock()
        observer_mock.on_detach_effect.side_effect = Exception()

        effect.register(observer_mock)
        assert observer_mock in effect._observers
        with patch("deepsim.visual_effects.abs_effect.EffectManager") as effect_manager_mock:
            effect.attach()
            with self.assertRaises(DeepSimCallbackError):
                effect.detach()
            effect_manager_mock.get_instance.return_value.discard.assert_called_once_with(effect)
            effect.mock.on_detach_effect.assert_called_once()
            observer_mock.on_detach_effect.assert_called_once_with(effect)

    def test_update(self):
        effect = DummyEffect()
        delta_time_mock = MagicMock()
        sim_time_mock = MagicMock()

        effect.update(delta_time=delta_time_mock,
                      sim_time=sim_time_mock)

        # Effect should be initialize on first update
        assert effect.is_initialized

        effect.mock.lazy_init.assert_called_once()
        effect.mock.on_update_effect.assert_called_once_with(delta_time_mock,
                                                             sim_time_mock)

