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

from deepsim.domain_randomizations.abs_randomizer import AbstractRandomizer


class DummyRandomizer(AbstractRandomizer):
    def __init__(self):
        self.mock = MagicMock()
        super(DummyRandomizer, self).__init__()

    def _randomize(self) -> None:
        self.mock.randomize()


class AbstractEffectTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_randomize(self):
        randomizer = DummyRandomizer()
        randomizer.randomize()
        randomizer.mock.randomize.assert_called_once()
