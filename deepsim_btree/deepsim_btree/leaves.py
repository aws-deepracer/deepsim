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
"""Classes for leaf nodes"""
from deepsim_btree.behaviour import Behaviour
import deepsim_btree.constants as const


class Success(Behaviour):
    """
    Success behaviour class that always returns SUCCESS
    """
    def update(self) -> const.Status:
        """
        Returns SUCCESS.

        Returns:
            const.Status: SUCCESS
        """
        return const.Status.SUCCESS


class Failure(Behaviour):
    """
    Failure behaviour class that always returns FAILURE
    """
    def update(self) -> const.Status:
        """
        Returns FAILURE.

        Returns:
            const.Status: FAILURE
        """
        return const.Status.FAILURE


class Running(Behaviour):
    """
    Running behaviour class that always returns RUNNING
    """
    def update(self) -> const.Status:
        """
        Returns RUNNING.

        Returns:
            const.Status: RUNNING
        """
        return const.Status.RUNNING
