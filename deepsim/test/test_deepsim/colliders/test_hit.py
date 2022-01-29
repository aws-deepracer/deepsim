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

from deepsim.colliders.hit import Hit


class HitTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_lt(self):
        a = Hit(obj=MagicMock(), ray=MagicMock(), entry=10.0, exit=15.0)
        b = Hit(obj=MagicMock(), ray=MagicMock(), entry=12.0, exit=14.0)
        assert a < b

    def test_gt(self):
        a = Hit(obj=MagicMock(), ray=MagicMock(), entry=13.0, exit=14.0)
        b = Hit(obj=MagicMock(), ray=MagicMock(), entry=12.0, exit=15.0)
        assert a > b
