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
"""A class for sequence node."""
from typing import Optional, Iterable, Iterator
from deepsim_btree.behaviour import Behaviour
from deepsim_btree.composites.composite import Composite
import deepsim_btree.constants as const


class Sequence(Composite):
    """
    Sequence behaviour that executes the children behaviour in order.
    """
    def __init__(self,
                 children: Optional[Iterable[Behaviour]] = None,
                 name: Optional[str] = None) -> None:
        """
        Initialize Sequence class.

        Args:
            children (Optional[Iterable[Behaviour]]): behaviour children
            name (Optional[str]): the name of this behaviour.
        """
        super(Sequence, self).__init__(children=children,
                                       name=name)
        self._current_child = None

    @property
    def current_child(self) -> Behaviour:
        """
        Returns the child that is currently executing.

        Returns:
            Behaviour: the child that is currently executing.
        """
        return self._current_child

    def _tick(self) -> Iterator[Behaviour]:
        """
         Process single tick for current behaviour.
         - If all child succeeds then behaviour stops with SUCCESS status
         - If any child fails then behaviour stops with FAILURE status

         Returns:
             Iterator[Behaviour]: the iterator to process the ticks on behaviour.
         """
        if self.status != const.Status.RUNNING:
            self._current_child = self.children[0] if self.children else None
            self.initialize()

        # customized update
        self.update()

        if not self.children:
            self.stop(const.Status.SUCCESS)
            yield self
            return

        self._status = const.Status.RUNNING
        for child in self.children:
            self._current_child = child
            while True:
                status = child.tick()
                if status == const.Status.SUCCESS:
                    break
                elif status == const.Status.RUNNING:
                    yield self
                else:
                    self.stop(const.Status.FAILURE)
                    yield self
                    return
        self.stop(const.Status.SUCCESS)
        yield self
