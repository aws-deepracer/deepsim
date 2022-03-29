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

import math
from deepsim.core.frustum import Frustum


myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class FrustumTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self):
        near = 1.0
        far = 100.0
        horizontal_fov = math.pi * 2.0 / 3.0
        view_ratio = 480.0 / 640.0
        frustum = Frustum(near=near,
                          far=far,
                          horizontal_fov=horizontal_fov,
                          view_ratio=view_ratio)
        assert frustum.near == 1.0
        assert frustum.far == far
        assert frustum.horizontal_fov == horizontal_fov
        assert frustum.view_ratio == view_ratio

    def test_setter(self):
        near = 1.0
        far = 100.0
        horizontal_fov = math.pi * 2.0 / 3.0
        view_ratio = 480.0 / 640.0
        frustum = Frustum(near=near,
                          far=far,
                          horizontal_fov=horizontal_fov,
                          view_ratio=view_ratio)
        assert frustum.near == 1.0
        assert frustum.far == far
        assert frustum.horizontal_fov == horizontal_fov
        assert frustum.view_ratio == view_ratio

        new_near = 2.0
        new_far = 200.0
        new_horizontal_fov = math.pi * 3.0 / 4.0
        new_view_ratio = 500.0 / 640.0
        frustum.near = new_near
        assert frustum.near == new_near
        frustum.far = new_far
        assert frustum.far == new_far
        frustum.horizontal_fov = new_horizontal_fov
        assert frustum.horizontal_fov == new_horizontal_fov
        frustum.view_ratio = new_view_ratio
        assert frustum.view_ratio == new_view_ratio
        assert frustum._is_outdated

