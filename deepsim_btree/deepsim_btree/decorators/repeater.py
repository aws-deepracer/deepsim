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
"""A class for repeater node."""
from typing import Optional

from deepsim_btree.decorators.decorator import Decorator
from deepsim_btree.behaviour import Behaviour

import deepsim_btree.constants as const


class Repeater(Decorator):
    """
    Repeater behaviour that repeats the child behaviour for given number of repeat.
    """
    def __init__(self,
                 child: Behaviour,
                 name: Optional[str] = None,
                 repeat: int = 0):
        """
        Initialize Repeater class.

        Args:
            child (Behaviour): the child behaviour to be decorated
            name (Optional[str]):the name of this behaviour.
            repeat (int): the number of repeat.
        """
        super(Repeater, self).__init__(child=child,
                                       name=name)
        self._repeat = repeat
        self._run_count = 0

    @property
    def repeat(self) -> int:
        """
        Returns the number of repeat configured.

        Returns:
            int: the number of repeat configured
        """
        return self._repeat

    @repeat.setter
    def repeat(self, value: int) -> None:
        """
        Set new repeat number for the repeater.

        Args:
            value (int): new repeat number.

        Raises:
            RuntimeError: if behaviour is in RUNNING state and trying to modify the repeat number
        """
        if self.status == const.Status.RUNNING:
            raise RuntimeError("Cannot modify repeat during RUNNING state!")
        self._repeat = value

    @property
    def run_count(self) -> int:
        """
        Returns current number of run.

        Returns:
            int: current number of run.
        """
        return self._run_count

    def initialize(self) -> None:
        """
        Initialize when behaviour starts.
        - resets the run count.
        """
        self._run_count = 0

    def update(self) -> const.Status:
        """
        Returns RUNNING while child behaviour is running for number of repeat given.
        Once child behaviour runs for given number of repeat, returns child status.

        Returns:
            const.Status: RUNNING while child behaviour is running for number of repeat given.
                            Once child behaviour finished running for given number of repeat,
                            then returns child status.
        """
        self._feedback_message = "'{0}' ran {1} times, repeating for {2}".format(self.child.name,
                                                                                 self.run_count,
                                                                                 self.repeat)

        if self.child.status != const.Status.RUNNING:
            self._run_count += 1
            if self._repeat >= 0 and self._run_count > self._repeat:
                return self.child.status
        return const.Status.RUNNING
