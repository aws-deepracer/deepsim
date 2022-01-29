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
"""A class for Behaviour."""
import abc
from typing import Optional, Iterator, Union
import uuid

import deepsim_btree.constants as const

# Python 2 and 3 compatible Abstract class
ABC = abc.ABCMeta('ABC', (object,), {})


class Behaviour(ABC):
    """
    The parent of all behaviour class.
    """
    def __init__(self, name: Optional[str] = None) -> None:
        """
        Initialize behaviour class.

        Args:
            name (Optional[str]):the name of this behaviour.
        """
        name = name if name else self.__class__.__name__
        if not isinstance(name, str):
            raise TypeError("behaviour name must be a string, but received {}".format(type(name)))
        self._name = name
        self._id = uuid.uuid4()
        self._feedback_message = ""
        self._status = const.Status.INVALID
        self._parent = None
        self._iterator = self._tick()

    @property
    def name(self) -> str:
        """
        Returns the name of behaviour.

        Returns:
            str: the name of behaviour.
        """
        return self._name

    @property
    def id(self) -> uuid.UUID:
        """
        Returns the id (uuid) of the behaviour.

        Returns:
            uuid.UUID: the id (uuid) of the behaviour.
        """
        return self._id

    @property
    def feedback_message(self) -> str:
        """
        Returns the feedback message. (debugging purpose)

        Returns:
            str: the feedback message.
        """
        return self._feedback_message

    @property
    def status(self) -> const.Status:
        """
        Returns the status of the behaviour.

        Returns:
            const.Status: the status of the behaviour.
        """
        return self._status

    @property
    def parent(self) -> Union['Behaviour', None]:
        """
        Returns the parent of this behaviour.

        Returns:
            Union['Behaviour', None]: the parent of this behaviour.
        """
        return self._parent

    @parent.setter
    def parent(self, value: 'Behaviour') -> None:
        """
        Sets the new parent of this behaviour.

        Args:
            value (Behaviour): the new parent of this behaviour.
        """
        self._parent = value

    def initialize(self) -> None:
        """
        Callback when entering the behaviour if it was not previously Status.RUNNING.
        - Subclass can implement this to initilize the behaviour before it starts.
        """
        pass

    def terminate(self, new_status: const.Status) -> None:
        """
        Callback when behaviour finishes execution.
        * RUNNING -> FAILURE or SUCCESS

        Args:
            new_status (const.Status): new status of behaviour
        """
        pass

    def update(self) -> const.Status:
        """
        Decision logic on the behaviour's new status.
        - Subclass should implement this to return the status of behaviour.

        Returns:
            const.Status: new status of the behaviour.
        """
        return const.Status.INVALID

    def tick(self) -> const.Status:
        """
        A direct means of calling single tick on this object without
        using the generator mechanism.
        - Calls single internal _tick with iterator.
        - If iterator raises StopIteration meaning at the end of iteration,
            it starts new iteration.

        Returns:
            const.Status: the status of this behaviour.
        """
        try:
            node = next(self._iterator)
        except StopIteration:
            self._iterator = self._tick()
            node = next(self._iterator)
        return node.status

    def _tick(self) -> Iterator['Behaviour']:
        """
        Internal tick that returns iterator.

        Returns:
            Iterator['Behaviour']: the iterator to process the ticks on behaviour.
        """
        if self.status != const.Status.RUNNING:
            self.initialize()
        new_state = self.update()
        if new_state not in const.Status:
            new_state = const.Status.INVALID
        if new_state != const.Status.RUNNING:
            self.stop(new_state)
        self._status = new_state
        yield self

    def stop(self, new_status=const.Status.INVALID) -> None:
        """
        Stops the behaviour and calls callback terminate.
        - sets new iterator for next tick call.

        Args:
            new_status (const.Status): the final status after behaviour stops.
        """
        self.terminate(new_status=new_status)
        self._status = new_status
        self._iterator = self._tick()
