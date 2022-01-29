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
"""A class for composite node."""
from typing import Optional, Iterable, List
import uuid

from deepsim_btree.behaviour import Behaviour
import deepsim_btree.constants as const


class Composite(Behaviour):
    """
    The parent class to all composite behaviours.
    - behaviour with children. (ex. selector/sequence)
    """
    def __init__(self,
                 children: Optional[Iterable[Behaviour]] = None,
                 name: Optional[str] = None) -> None:
        """
        Initialize Composite class.

        Args:
            children (Optional[Iterable[Behaviour]]): behaviour children
            name (Optional[str]): the name of this behaviour.
        """
        super(Composite, self).__init__(name=name)
        self._children = []
        children = list(children) if children else []
        self.add_children(children=children)

    @property
    def children(self) -> List[Behaviour]:
        """
        Returns the children of this composite behaviour.

        Returns:
            List[Behavior]: children in iterable data-structure.
        """
        return self._children

    def stop(self, new_status: const.Status = const.Status.INVALID) -> None:
        """
        Stop the behaviour with given status as final status.

        Args:
            new_status (const.Status): the final status of the behaviour after stop.
        """
        for child in self._children:
            if child.status == const.Status.RUNNING:
                child.stop()
        super(Composite, self).stop(new_status=new_status)

    def add_child(self, child: Behaviour) -> uuid.UUID:
        """
        Add new behaviour to children list.

        Args:
            child (Behaviour): new child to add to children list.

        Returns:
            uuid.UUID: the uuid of new child.

        Raises:
            RuntimeError: if behaviour is in RUNNING state and trying to modify the children list or
                given child already has a parent.
            TypeError: if given child is not Behaviour type.
        """
        if self.status == const.Status.RUNNING:
            raise RuntimeError("Cannot modify children during RUNNING state!")
        if not isinstance(child, Behaviour):
            raise TypeError("child must be a type of Behaviour, but received {}".format(type(child)))
        if child.parent is not None:
            raise RuntimeError("behaviour '{}' already has parent '{}'".format(child.name,
                                                                               child.parent.name))
        self._children.append(child)
        child.parent = self
        return child.id

    def add_children(self, children: Iterable[Behaviour]) -> 'Composite':
        """
        Add given children to existing children list.

        Args:
            children (Iterable[Behaviour]): new children to add to existing list.

        Returns:
            Composite: self
        """
        for child in children:
            self.add_child(child=child)
        return self

    def remove_child(self, child: Behaviour) -> int:
        """
        Remove given child from the children list.

        Args:
            child (Behaviour): the behaviour to remove from children list.

        Returns:
            int: the index of child originally located in the list.

        Raises:
            RuntimeError: if behaviour is in RUNNING state and trying to modify the children list
        """
        if self.status == const.Status.RUNNING:
            raise RuntimeError("Cannot modify children during RUNNING state!")

        if child.status == const.Status.RUNNING:
            child.stop()
        child_idx = self._children.index(child)
        self._children.remove(child)
        child.parent = None
        return child_idx

    def remove_all_children(self) -> None:
        """
        Remove all children from the children list.

        Raises:
            RuntimeError: if behaviour is in RUNNING state and trying to modify the children list
        """
        if self.status == const.Status.RUNNING:
            raise RuntimeError("Cannot modify children during RUNNING state!")

        for child in self._children:
            if child.status == const.Status.RUNNING:
                child.stop()
            child.parent = None
        self._children = []

    def replace_child(self, child: Behaviour, replacement: Behaviour) -> int:
        """
        Replace given child with replacement.

        Args:
            child (Behaviour): child to be replaced.
            replacement (Behaviour): replacement for given child.

        Returns:
            int: the index of child originally located in the list.

        Raises:
            RuntimeError: if behaviour is in RUNNING state and trying to modify the children list
            TypeError: if given child or replacement is not Behaviour type.
        """
        if self.status == const.Status.RUNNING:
            raise RuntimeError("Cannot modify children during RUNNING state!")
        if not isinstance(child, Behaviour):
            raise TypeError("child must be a type of Behaviour, but received {}".format(type(child)))
        if not isinstance(replacement, Behaviour):
            raise TypeError("replacement must be a type of Behaviour, but received {}".format(type(child)))

        child_index = self.remove_child(child)
        self.insert_child(replacement, child_index)
        return child_index

    def remove_child_by_id(self, child_id: uuid.UUID) -> None:
        """
        Remove a child with given uuid.

        Args:
            child_id (uuid.UUID): the uuid of child to be removed.

        Raises:
            RuntimeError: if behaviour is in RUNNING state and trying to modify the children list
            IndexError: if child_id not found in the children list.
        """
        if self.status == const.Status.RUNNING:
            raise RuntimeError("Cannot modify children during RUNNING state!")

        child_list = [child for child in self._children if child.id == child_id]
        if child_list:
            self.remove_child(child_list[0])
        else:
            raise IndexError('child was not found with specified id [{}]'.format(child_id))

    def prepend_child(self, child: Behaviour) -> uuid.UUID:
        """
        Prepend the child to the beginning of the children list.

        Args:
            child (Behaviour): the child to prepend.

        Returns:
            uuid.UUID: the uuid of given child.

        Raises:
            RuntimeError: if behaviour is in RUNNING state and trying to modify the children list or
                given child already has a parent.
            TypeError: if given child is not Behaviour type.
        """
        if self.status == const.Status.RUNNING:
            raise RuntimeError("Cannot modify children during RUNNING state!")

        if not isinstance(child, Behaviour):
            raise TypeError("child must be a type of Behaviour, but received {}".format(type(child)))
        if child.parent is not None:
            raise RuntimeError("behaviour '{}' already has parent '{}'".format(child.name,
                                                                               child.parent.name))
        self._children.insert(0, child)
        child.parent = self
        return child.id

    def insert_child(self, child: Behaviour, index: int) -> uuid.UUID:
        """
        Insert the given child at given index of children list.

        Args:
            child (Behaviour): the child to be inserted.
            index (int): the index of children list to insert given child.

        Returns:
            uuid.UUID: the uuid of given child.

        Raises:
            RuntimeError: if behaviour is in RUNNING state and trying to modify the children list or
                given child already has a parent.
            TypeError: if given child is not Behaviour type.
        """
        if self.status == const.Status.RUNNING:
            raise RuntimeError("Cannot modify children during RUNNING state!")

        if not isinstance(child, Behaviour):
            raise TypeError("child must be a type of Behaviour, but received {}".format(type(child)))
        if child.parent is not None:
            raise RuntimeError("behaviour '{}' already has parent '{}'".format(child.name,
                                                                               child.parent.name))
        self._children.insert(index, child)
        child.parent = self
        return child.id
