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

from deepsim.domain_randomizations.randomizer_manager import RandomizerManager


myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class RandomizerManagerTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_add(self):
        randomizer_manager = RandomizerManager(is_singleton=False)
        randomizer = MagicMock()

        randomizer_manager.add(randomizer)
        assert randomizer in randomizer_manager._randomizers

    def test_remove(self):
        randomizer_manager = RandomizerManager(is_singleton=False)
        randomizer = MagicMock()

        randomizer_manager.add(randomizer)

        randomizer_manager.remove(randomizer)
        assert randomizer not in randomizer_manager._randomizers

    def test_remove_non_existing_randomizer(self):
        randomizer_manager = RandomizerManager(is_singleton=False)
        randomizer = MagicMock()

        with self.assertRaises(KeyError):
            randomizer_manager.remove(randomizer)

    def test_discard(self):
        randomizer_manager = RandomizerManager(is_singleton=False)
        randomizer = MagicMock()

        randomizer_manager.add(randomizer)

        randomizer_manager.discard(randomizer)
        assert randomizer not in randomizer_manager._randomizers

    def test_discard_non_existing_randomizer(self):
        randomizer_manager = RandomizerManager(is_singleton=False)
        randomizer = MagicMock()

        randomizer_manager.discard(randomizer)

    def test_randomize(self):
        randomizer_manager = RandomizerManager(is_singleton=False)
        randomizer = MagicMock()

        randomizer_manager.add(randomizer)
        randomizer_manager.randomize()
        randomizer.randomize.assert_called_once()
