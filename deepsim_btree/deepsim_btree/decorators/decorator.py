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
"""A class for decorator node."""
from typing import Optional, Iterator

from deepsim_btree.behaviour import Behaviour
import deepsim_btree.constants as const


class Decorator(Behaviour):
    """
    The parent class to all decorator behaviours.
    - behaviour with single child (ex. condition/limit/etc.)
    """
    def __init__(self,
                 child: Behaviour,
                 name: Optional[str] = None) -> None:
        """
        Initialize Decorator class.

        Args:
            child (Behaviour): behaviour child to decorate
            name (Optional[str]): the name of this behaviour.
        """
        super(Decorator, self).__init__(name=name)
        if not isinstance(child, Behaviour):
            raise TypeError("child must be a Behaviour, but received {}".format(type(child)))
        self._child = child
        self._child.parent = self

    @property
    def child(self) -> Behaviour:
        """
        Returns the child that this decorator behaviour is decorating.

        Returns:
            Behaviour: the child that this decorator behaviour is decorating.
        """
        return self._child

    @child.setter
    def child(self, value: Behaviour) -> None:
        """
        Set new child for the decorator.

        Args:
            value (Behaviour): new child behaviour.

        Raises:
            RuntimeError: if behaviour is in RUNNING state and trying to modify the child or
                given child already has a parent.
            TypeError: if given child is not Behaviour type.
        """
        if self.status == const.Status.RUNNING:
            raise RuntimeError("Cannot modify children during RUNNING state!")

        if not isinstance(value, Behaviour):
            raise TypeError("child must be a Behaviour, but received {}".format(type(value)))
        if value.parent is not None:
            raise RuntimeError("behaviour '{}' already has parent '{}'".format(value.name,
                                                                               value.parent.name))
        if self._child:
            self._child.parent = None
        self._child = value
        self._child.parent = self

    def _tick(self) -> Iterator['Behaviour']:
        """
        Process single tick for current behaviour.
        - Process single child behaviour tick and
            stops the behaviour based on decorated status.

        Returns:
            Iterator['Behaviour']: the iterator to process the ticks on behaviour.
        """
        if self.status != const.Status.RUNNING:
            self.initialize()

        self._status = const.Status.RUNNING

        while True:
            _ = self._child.tick()

            new_status = self.update()
            if new_status not in const.Status:
                new_status = const.Status.INVALID
            if new_status != const.Status.RUNNING:
                self.stop(new_status=new_status)
                yield self
                return
            yield self

    def stop(self, new_status=const.Status.INVALID) -> None:
        """
        Stop the decorator behaviour along with child behaviour if child is running.

        Args:
            new_status (const.Status): the final status after stopping.
        """
        if self._child.status == const.Status.RUNNING:
            self._child.stop()
        super(Decorator, self).stop(new_status=new_status)
