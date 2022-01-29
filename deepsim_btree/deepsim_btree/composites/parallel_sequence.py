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
"""A class for parallel sequence node."""
from typing import Optional, Iterable, Iterator
from deepsim_btree.behaviour import Behaviour
from deepsim_btree.composites.composite import Composite
import deepsim_btree.constants as const


class ParallelSequence(Composite):
    """
    Sequence behaviour that executes the behaviour children in parallel.
    """

    def _tick(self) -> Iterator['Behaviour']:
        """
        Process single tick for current behaviour.
        - If all child succeeds then behaviour stops with SUCCESS status
        - If any child fails then behaviour stops with FAILURE status

        Returns:
            Iterator['Behaviour']: the iterator to process the ticks on behaviour.
        """
        if self.status != const.Status.RUNNING:
            self.initialize()

        # customized update
        self.update()

        if not self.children:
            self.stop(const.Status.SUCCESS)
            yield self
            return

        self._status = const.Status.RUNNING
        children = list(self.children)
        while children:
            success_children = []
            for child in children:
                status = child.tick()
                if status == const.Status.SUCCESS:
                    success_children.append(child)
                elif status != const.Status.RUNNING:
                    self.stop(const.Status.FAILURE)
                    yield self
                    return
            for success_child in success_children:
                children.remove(success_child)
            if not children:
                break
            yield self

        self.stop(const.Status.SUCCESS)
        yield self
