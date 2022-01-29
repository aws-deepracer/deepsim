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
"""Module to contain domain randomization related constants"""
from enum import Enum


class ModelRandomizerType(Enum):
    """ Model Randomizer Type

    MODEL type will randomize the color of overall model.
    LINK type will randomize the color for each link.
    VISUAL type will randomize the color for each link's visual
    """
    MODEL = "model"
    LINK = "link"
    VISUAL = "visual"


class RangeType(Enum):
    """Range Type"""
    COLOR = 'color'
    ATTENUATION = 'attenuation'


RANGE_MIN = 'min'
RANGE_MAX = 'max'


class ColorAttr(object):
    """Color attributes"""
    R = 'r'
    G = 'g'
    B = 'b'


class Attenuation(object):
    """Light attenuation attributes"""
    CONSTANT = 'constant'
    LINEAR = 'linear'
    QUADRATIC = 'quadratic'
