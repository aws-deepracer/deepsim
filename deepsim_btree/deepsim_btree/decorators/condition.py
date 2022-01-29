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
"""A class for condition node."""
from typing import Optional

from deepsim_btree.decorators.decorator import Decorator
from deepsim_btree.behaviour import Behaviour

import deepsim_btree.constants as const


class Condition(Decorator):
    """
    Condition behaviour that returns SUCCESS when child returns target status,
    and returns RUNNING for all other status.
    """
    def __init__(self,
                 child: Behaviour,
                 name: Optional[str] = None,
                 success_status: const.Status = const.Status.SUCCESS) -> None:
        """
        Initialize Condition class.

        Args:
            child (Behaviour): the child behaviour to be decorated
            name (Optional[str]):the name of this behaviour.
            success_status (const.Status): target success status.
        """
        super(Condition, self).__init__(child=child,
                                        name=name)
        self._success_status = success_status

    @property
    def success_status(self) -> const.Status:
        """
        Returns the target success status.

        Returns:
            const.Status: the target success status.
        """
        return self._success_status

    @success_status.setter
    def success_status(self, value: const.Status) -> None:
        """
        Set new success status for the condition.

        Args:
            value (const.Status): new success status.

        Raises:
            RuntimeError: if behaviour is in RUNNING state and trying to modify the success_status
        """
        if self.status == const.Status.RUNNING:
            raise RuntimeError("Cannot modify success_status during RUNNING state!")
        self._success_status = value

    def update(self) -> const.Status:
        """
        Returns SUCCESS if child status is same as target success status.
        Otherwise returns RUNNING.

        Returns:
            const.Status: SUCCESS if child status is same as target success status.
                            Otherwise returns RUNNING.
        """
        self._feedback_message = "'{0}' has status {1}, waiting for {2}".format(self.child.name,
                                                                                self.child.status,
                                                                                self.success_status)
        if self.child.status == self.success_status:
            return const.Status.SUCCESS
        return const.Status.RUNNING
