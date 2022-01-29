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
"""Classes for status flipper nodes."""
from deepsim_btree.decorators.decorator import Decorator

import deepsim_btree.constants as const


class Inverter(Decorator):
    """
    Inverter behaviour class that returns inverted child status.
    """
    def update(self) -> const.Status:
        """
        Returns SUCCESS if child status is FAILURE,
            returns FAILURE if child status is SUCCESS,
            and returns RUNNING if child status is RUNNING
        Returns:
            const.Status: SUCCESS if child status is FAILURE,
                          FAILURE if child status is SUCCESS,
                          and RUNNING if child status is RUNNING
        """
        if self.child.status == const.Status.SUCCESS:
            self._feedback_message = "success -> failure"
            return const.Status.FAILURE
        elif self.child.status == const.Status.FAILURE:
            self._feedback_message = "failure -> success"
            return const.Status.SUCCESS
        self._feedback_message = self.child.feedback_message
        return self.child.status


class Succeeder(Decorator):
    """
    Succeeder behaviour class that returns SUCCESS regardless of child finish status.
    """
    def update(self) -> const.Status:
        """
        Returns RUNNING if child is RUNNING, otherwise returns SUCCESS.
        Returns:
            const.Status: RUNNING if child is RUNNING, otherwise returns SUCCESS.
        """
        if self.child.status != const.Status.RUNNING:
            return const.Status.SUCCESS
        return const.Status.RUNNING


class UntilFail(Decorator):
    """
    UntilFail behaviour class that runs the child behaviour until fails.
    """
    def update(self) -> const.Status:
        """
        Returns SUCCESS when child status is FAILURE, otherwise returns RUNNING.

        Returns:
            const.Status: SUCCESS when child status is FAILURE, otherwise returns RUNNING.
        """
        if self.child.status == const.Status.FAILURE:
            return const.Status.SUCCESS
        return const.Status.RUNNING


class RunningIsFailure(Decorator):
    """
    RunningIsFailure behaviour class that returns FAILURE when child is RUNNING status.
    """
    def update(self) -> const.Status:
        """
        Returns FAILURE if child is RUNNING status, otherwise returns child status.
        Returns:
            const.Status: FAILURE if child is RUNNING status, otherwise returns child status.
        """
        if self.child.status == const.Status.RUNNING:
            self._feedback_message = "running is failure ({})".format(self.child.feedback_message)
            return const.Status.FAILURE
        else:
            self._feedback_message = self.child.feedback_message
            return self.child.status


class RunningIsSuccess(Decorator):
    """
    RunningIsSuccess behaviour class that returns SUCCESS when child is RUNNING status.
    """
    def update(self) -> const.Status:
        """
        Returns SUCCESS if child is RUNNING status, otherwise returns child status.
        Returns:
            const.Status: SUCCESS if child is RUNNING status, otherwise returns child status.
        """
        if self.child.status == const.Status.RUNNING:
            self._feedback_message = "running is success ({})".format(self.child.feedback_message)
            return const.Status.SUCCESS
        else:
            self._feedback_message = self.child.feedback_message
            return self.child.status


class FailureIsSuccess(Decorator):
    """
    FailureIsSuccess behaviour class that returns FAILURE when child is SUCCESS status.
    """
    def update(self) -> const.Status:
        """
        Returns FAILURE if child is SUCCESS status, otherwise returns child status.
        Returns:
            const.Status: FAILURE if child is SUCCESS status, otherwise returns child status.
        """
        if self.child.status == const.Status.FAILURE:
            self._feedback_message = "failure is success ({})".format(self.child.feedback_message)
            return const.Status.SUCCESS
        else:
            self._feedback_message = self.child.feedback_message
            return self.child.status


class FailureIsRunning(Decorator):
    """
    FailureIsRunning behaviour class that returns RUNNING when child is FAILURE status.
    """
    def update(self) -> const.Status:
        """
        Returns RUNNING if child is FAILURE status, otherwise returns child status.
        Returns:
            const.Status: RUNNING if child is FAILURE status, otherwise returns child status.
        """
        if self.child.status == const.Status.FAILURE:
            self._feedback_message = "failure is running ({})".format(self.child.feedback_message)
            return const.Status.RUNNING
        else:
            self._feedback_message = self.child.feedback_message
            return self.child.status


class SuccessIsFailure(Decorator):
    """
    SuccessIsFailure behaviour class that returns FAILURE when child is SUCCESS status.
    """
    def update(self) -> const.Status:
        """
        Returns FAILURE if child is SUCCESS status, otherwise returns child status.
        Returns:
            const.Status: FAILURE if child is SUCCESS status, otherwise returns child status.
        """
        if self.child.status == const.Status.SUCCESS:
            self._feedback_message = "success is failure ({})".format(self.child.feedback_message)
            return const.Status.FAILURE
        else:
            self._feedback_message = self.child.feedback_message
            return self.child.status


class SuccessIsRunning(Decorator):
    """
    SuccessIsRunning behaviour class that returns RUNNING when child is SUCCESS status.
    """
    def update(self) -> const.Status:
        """
        Returns RUNNING if child is SUCCESS status, otherwise returns child status.
        Returns:
            const.Status: RUNNING if child is SUCCESS status, otherwise returns child status.
        """
        if self.child.status == const.Status.SUCCESS:
            self._feedback_message = "success is running ({})".format(self.child.feedback_message)
            return const.Status.RUNNING
        else:
            self._feedback_message = self.child.feedback_message
            return self.child.status
