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

from deepsim.math.material import Material
from deepsim.math.color import Color

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class MaterialTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_initialize(self):
        material = Material()

        assert material.ambient == Color()
        assert material.diffuse == Color()
        assert material.specular == Color()
        assert material.emissive == Color()

        ambient = Color(0.0, 0.1, 0.2, 0.3)
        diffuse = Color(0.1, 0.2, 0.3, 0.4)
        specular = Color(0.2, 0.3, 0.4, 0.5)
        emissive = Color(0.3, 0.4, 0.5, 0.6)

        material = Material(ambient=ambient,
                            diffuse=diffuse,
                            specular=specular,
                            emissive=emissive)

        assert material.ambient == ambient
        assert material.diffuse == diffuse
        assert material.specular == specular
        assert material.emissive == emissive

    def test_setters(self):
        material = Material()

        assert material.ambient == Color()
        assert material.diffuse == Color()
        assert material.specular == Color()
        assert material.emissive == Color()

        ambient = Color(0.0, 0.1, 0.2, 0.3)
        diffuse = Color(0.1, 0.2, 0.3, 0.4)
        specular = Color(0.2, 0.3, 0.4, 0.5)
        emissive = Color(0.3, 0.4, 0.5, 0.6)

        material.ambient = ambient
        material.diffuse = diffuse
        material.specular = specular
        material.emissive = emissive

        assert material.ambient == ambient
        assert material.diffuse == diffuse
        assert material.specular == specular
        assert material.emissive == emissive

    def test_copy(self):
        ambient = Color(0.0, 0.1, 0.2, 0.3)
        diffuse = Color(0.1, 0.2, 0.3, 0.4)
        specular = Color(0.2, 0.3, 0.4, 0.5)
        emissive = Color(0.3, 0.4, 0.5, 0.6)

        material = Material(ambient=ambient,
                            diffuse=diffuse,
                            specular=specular,
                            emissive=emissive)

        material_copy = material.copy()
        assert material_copy == material
        assert material_copy.ambient == material.ambient
        assert material_copy.ambient is not material.ambient
        assert material_copy.diffuse == material.diffuse
        assert material_copy.diffuse is not material.diffuse
        assert material_copy.specular == material.specular
        assert material_copy.specular is not material.specular
        assert material_copy.emissive == material.emissive
        assert material_copy.emissive is not material.emissive

    def test_eq(self):
        ambient = Color(0.0, 0.1, 0.2, 0.3)
        diffuse = Color(0.1, 0.2, 0.3, 0.4)
        specular = Color(0.2, 0.3, 0.4, 0.5)
        emissive = Color(0.3, 0.4, 0.5, 0.6)

        material = Material(ambient=ambient,
                            diffuse=diffuse,
                            specular=specular,
                            emissive=emissive)

        material2 = Material(ambient=ambient,
                             diffuse=diffuse,
                             specular=specular,
                             emissive=emissive)

        assert material == material2
        assert material.ambient == material2.ambient
        assert material.ambient is not material2.ambient
        assert material.diffuse == material2.diffuse
        assert material.diffuse is not material2.diffuse
        assert material.specular == material2.specular
        assert material.specular is not material2.specular
        assert material.emissive == material2.emissive
        assert material.emissive is not material2.emissive



