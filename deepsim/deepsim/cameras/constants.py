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
"""Module to contain DeepSim camera related constants"""
from enum import Enum, unique


@unique
class CameraSettings(Enum):
    """This enum is used to index into the camera settings list"""
    HORZ_FOV = 1
    PADDING_PCT = 2
    IMG_WIDTH = 3
    IMG_HEIGHT = 4

    @classmethod
    def get_empty_dict(cls) -> dict:
        """Returns dictionary with the enum as key values and None's as the values, clients
           are responsible for populating the dict accordingly
        """
        empty_dict = dict()
        for val in cls._value2member_map_.values():
            empty_dict[val] = None
        return empty_dict
