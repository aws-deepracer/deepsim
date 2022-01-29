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
from typing import Any, Callable, Optional, Iterable
from unittest import mock, TestCase
from unittest.mock import patch, MagicMock, call
import inspect

from deepsim_btree.decorators.status_flippers import (
    Inverter, Succeeder, UntilFail,
    RunningIsFailure, RunningIsSuccess,
    SuccessIsRunning, SuccessIsFailure,
    FailureIsSuccess, FailureIsRunning
)
from deepsim_btree.constants import Status
from deepsim_btree.leaves import Success, Failure, Running

myself: Callable[[], Any] = lambda: inspect.stack()[1][3]


class StatusFlippersTest(TestCase):
    def setUp(self) -> None:
        pass

    def test_inverter(self):
        child = Success()
        inverter = Inverter(child=child)
        assert inverter.name == "Inverter"

        status = inverter.tick()
        assert status == Status.FAILURE

        child = Failure()
        inverter = Inverter(child=child)
        status = inverter.tick()
        assert status == Status.SUCCESS

        child = Running()
        inverter = Inverter(child=child)
        status = inverter.tick()
        assert status == Status.RUNNING

    def test_succeeder(self):
        child = Success()
        succeeder = Succeeder(child=child)
        assert succeeder.name == "Succeeder"

        status = succeeder.tick()
        assert status == Status.SUCCESS

        child = Failure()
        succeeder = Succeeder(child=child)
        status = succeeder.tick()
        assert status == Status.SUCCESS

    def test_until_fail(self):
        child = Success()
        until_fail = UntilFail(child=child)
        assert until_fail.name == "UntilFail"

        status = until_fail.tick()
        assert status == Status.RUNNING
        status = until_fail.tick()
        assert status == Status.RUNNING

        child = Failure()
        until_fail = UntilFail(child=child)
        status = until_fail.tick()
        assert status == Status.SUCCESS

    def test_running_is_failure(self):
        child = Success()
        running_is_failure = RunningIsFailure(child=child)
        assert running_is_failure.name == "RunningIsFailure"

        status = running_is_failure.tick()
        assert status == Status.SUCCESS

        child = Failure()
        running_is_failure = RunningIsFailure(child=child)
        status = running_is_failure.tick()
        assert status == Status.FAILURE

        child = Running()
        running_is_failure = RunningIsFailure(child=child)
        status = running_is_failure.tick()
        assert status == Status.FAILURE

    def test_running_is_success(self):
        child = Success()
        running_is_success = RunningIsSuccess(child=child)
        assert running_is_success.name == "RunningIsSuccess"

        status = running_is_success.tick()
        assert status == Status.SUCCESS

        child = Failure()
        running_is_success = RunningIsSuccess(child=child)
        status = running_is_success.tick()
        assert status == Status.FAILURE

        child = Running()
        running_is_success = RunningIsSuccess(child=child)
        status = running_is_success.tick()
        assert status == Status.SUCCESS

    def test_success_is_running(self):
        child = Success()
        success_is_running = SuccessIsRunning(child=child)
        assert success_is_running.name == "SuccessIsRunning"

        status = success_is_running.tick()
        assert status == Status.RUNNING

        child = Failure()
        success_is_running = SuccessIsRunning(child=child)
        status = success_is_running.tick()
        assert status == Status.FAILURE

        child = Running()
        success_is_running = SuccessIsRunning(child=child)
        status = success_is_running.tick()
        assert status == Status.RUNNING

    def test_success_is_failure(self):
        child = Success()
        success_is_failure = SuccessIsFailure(child=child)
        assert success_is_failure.name == "SuccessIsFailure"

        status = success_is_failure.tick()
        assert status == Status.FAILURE

        child = Failure()
        success_is_failure = SuccessIsFailure(child=child)
        status = success_is_failure.tick()
        assert status == Status.FAILURE

        child = Running()
        success_is_failure = SuccessIsFailure(child=child)
        status = success_is_failure.tick()
        assert status == Status.RUNNING

    def test_failure_is_success(self):
        child = Success()
        failure_is_success = FailureIsSuccess(child=child)
        assert failure_is_success.name == "FailureIsSuccess"

        status = failure_is_success.tick()
        assert status == Status.SUCCESS

        child = Failure()
        failure_is_success = FailureIsSuccess(child=child)
        status = failure_is_success.tick()
        assert status == Status.SUCCESS

        child = Running()
        failure_is_success = FailureIsSuccess(child=child)
        status = failure_is_success.tick()
        assert status == Status.RUNNING

    def test_failure_is_running(self):
        child = Success()
        failure_is_running = FailureIsRunning(child=child)
        assert failure_is_running.name == "FailureIsRunning"

        status = failure_is_running.tick()
        assert status == Status.SUCCESS

        child = Failure()
        failure_is_running = FailureIsRunning(child=child)
        status = failure_is_running.tick()
        assert status == Status.RUNNING

        child = Running()
        failure_is_running = FailureIsRunning(child=child)
        status = failure_is_running.tick()
        assert status == Status.RUNNING
