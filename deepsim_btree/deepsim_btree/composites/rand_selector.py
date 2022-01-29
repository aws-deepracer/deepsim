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
"""A class for random selector node."""
import random
from typing import Optional, Iterable, Iterator, Union

from deepsim_btree.behaviour import Behaviour
from deepsim_btree.composites.composite import Composite
import deepsim_btree.constants as const


class RandomSelector(Composite):
    """
    Selector behaviour that executes the children behaviour in random order.
    """
    def __init__(self,
                 children: Optional[Iterable[Behaviour]] = None,
                 name: Optional[str] = None,
                 seed: Optional[int] = None) -> None:
        """
        Initialize RandomSelector class.

        Args:
            children (Optional[Iterable[Behaviour]]): behaviour children
            name (Optional[str]): the name of this behaviour.
            seed (Optional[int]): random seed
        """
        super(RandomSelector, self).__init__(children=children,
                                             name=name)
        self._current_child = None
        self._seed = seed

    @property
    def current_child(self) -> Behaviour:
        """
        Returns the child that is currently executing.

        Returns:
            Behaviour: the child that is currently executing.
        """
        return self._current_child

    @property
    def seed(self) -> Union[int, None]:
        """
        Returns current random seed.

        Returns:
            Union[int, None]: current random seed.
        """
        return self._seed

    @seed.setter
    def seed(self, value: Optional[int]) -> None:
        """
        Sets new random seed.

        Args:
            value (Optional[int]): new random seed.
        """
        self._seed = value

    def _tick(self) -> Iterator[Behaviour]:
        """
        Process single tick for current behaviour.
        - If any child succeeds then behaviour stops with SUCCESS status
        - If all child fails then behaviour stops with FAILURE status

        Returns:
            Iterator[Behaviour]: the iterator to process the ticks on behaviour.
        """
        random.seed(self.seed)
        shuffled_children = list(self.children)
        random.shuffle(shuffled_children)

        if self.status != const.Status.RUNNING:
            self._current_child = shuffled_children[0] if shuffled_children else None
            self.initialize()

        # customized update
        self.update()

        if not shuffled_children:
            self.stop(const.Status.FAILURE)
            yield self
            return

        self._status = const.Status.RUNNING
        for child in shuffled_children:
            self._current_child = child
            while True:
                status = child.tick()
                if status == const.Status.SUCCESS:
                    self.stop(const.Status.SUCCESS)
                    yield self
                    return
                elif status == const.Status.RUNNING:
                    yield self
                else:
                    break
        self.stop(const.Status.FAILURE)
        yield self
