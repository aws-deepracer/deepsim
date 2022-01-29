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
"""A class for limit node."""
from typing import Optional, Union

from deepsim_btree.decorators.decorator import Decorator
from deepsim_btree.behaviour import Behaviour

import deepsim_btree.constants as const


class Limit(Decorator):
    """
    Limit behaviour that returns child status,
    but fails and stop the child behavior when tick exceeds the tick limit given.
    """
    def __init__(self,
                 child: Behaviour,
                 name: Optional[str] = None,
                 tick_limit: Optional[int] = None):
        """
        Initialize Limit class.

        Args:
            child (Behaviour): the child behaviour to be decorated
            name (Optional[str]):the name of this behaviour.
            tick_limit Optional[int]: tick limit. if None or < 0 is given then no limit.
        """
        super(Limit, self).__init__(child=child,
                                    name=name)
        if tick_limit is not None and tick_limit < 0:
            tick_limit = None
        self._tick_limit = tick_limit
        self._tick_count = 0

    @property
    def tick_limit(self) -> Union[int, None]:
        """
        Returns the tick limit.

        Returns:
            Union[int, None]: the tick limit configured.
        """
        return self._tick_limit

    @tick_limit.setter
    def tick_limit(self, value: Union[int, None]) -> None:
        """
        Set new tick limit for the limit.

        Args:
            value (Union[int, None]): new tick limit.

        Raises:
            RuntimeError: if behaviour is in RUNNING state and trying to modify the tick limit
        """
        if self.status == const.Status.RUNNING:
            raise RuntimeError("Cannot modify tick limit during RUNNING state!")
        self._tick_limit = value

    @property
    def tick_count(self) -> int:
        """
        Returns current tick count.

        Returns:
            int: current tick count.
        """
        return self._tick_count

    def initialize(self) -> None:
        """
        Initialize when behaviour starts.
        - reset the tick count.
        """
        self._tick_count = 0

    def update(self) -> const.Status:
        """
        Returns child status if limit is not passed, but
        if tick exceeds the limit then returns FAILURE.

        Returns:
            const.Status: child status if limit is not passed.
                            if tick exceeds the limit then returns FAILURE.
        """
        self._tick_count += 1
        self._feedback_message = "'{0}' has tick_count {1}, running for {2}".format(self.child.name,
                                                                                    str(self.tick_count),
                                                                                    str(self.tick_limit))
        if self._tick_limit is not None and self._tick_count > self._tick_limit:
            return const.Status.FAILURE
        return self.child.status
