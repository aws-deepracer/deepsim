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
from typing import Any, Callable
from unittest import TestCase
from unittest.mock import patch, MagicMock, call
import inspect

from deepsim.ros.service_proxy_wrapper import ServiceProxyWrapper


@patch("deepsim.ros.service_proxy_wrapper.rospy")
class ServiceProxyWrapperTest(TestCase):
    def setUp(self) -> None:
        self.name = "test_service"
        self.service_class_mock = MagicMock()

    def test_initialize_default(self, rospy_mock):
        service_wrapper = ServiceProxyWrapper(name=self.name,
                                              service_class=self.service_class_mock)

        assert service_wrapper.max_retry_attempts == 5
        rospy_mock.wait_for_service.assert_called_once_with(self.name)
        rospy_mock.ServiceProxy.assert_called_once_with(name=self.name,
                                                        service_class=self.service_class_mock,
                                                        persistent=False,
                                                        headers=None)

    def test_initialize(self, rospy_mock):
        persistent_mock = MagicMock()
        headers_mock = MagicMock()
        service_wrapper = ServiceProxyWrapper(name=self.name,
                                              service_class=self.service_class_mock,
                                              persistent=persistent_mock,
                                              headers=headers_mock,
                                              should_wait_for_service=True,
                                              max_retry_attempts=3)

        assert service_wrapper.max_retry_attempts == 3
        rospy_mock.wait_for_service.assert_called_once_with(self.name)
        rospy_mock.ServiceProxy.assert_called_once_with(name=self.name,
                                                        service_class=self.service_class_mock,
                                                        persistent=persistent_mock,
                                                        headers=headers_mock)

    def test_initialize_no_wait(self, rospy_mock):
        _ = ServiceProxyWrapper(name=self.name,
                                service_class=self.service_class_mock,
                                should_wait_for_service=False)

        rospy_mock.wait_for_service.assert_not_called()
        rospy_mock.ServiceProxy.assert_called_once_with(name=self.name,
                                                        service_class=self.service_class_mock,
                                                        persistent=False,
                                                        headers=None)

    def test_max_retry_attempts(self, rospy_mock):
        service_wrapper = ServiceProxyWrapper(name=self.name,
                                              service_class=self.service_class_mock,
                                              should_wait_for_service=False)

        assert service_wrapper.max_retry_attempts == 5
        service_wrapper.max_retry_attempts = 3
        assert service_wrapper.max_retry_attempts == 3

    def test_call(self, rospy_mock):
        service_wrapper = ServiceProxyWrapper(name=self.name,
                                              service_class=self.service_class_mock)
        arg = MagicMock()
        service_wrapper(arg)
        service_wrapper(arg=arg)
        rospy_mock.ServiceProxy.return_value.has_calls(
            call(arg),
            call(arg=arg)
        )

    def test_call_exception(self, rospy_mock):
        rospy_mock.ServiceProxy.return_value.side_effect = Exception()

        service_wrapper = ServiceProxyWrapper(name=self.name,
                                              service_class=self.service_class_mock)
        arg = MagicMock()
        with self.assertRaises(Exception):
            service_wrapper(arg)
            assert rospy_mock.ServiceProxy.return_value.call_count == 5

    def test_call_typeerror(self, rospy_mock):
        rospy_mock.ServiceProxy.return_value.side_effect = TypeError()

        service_wrapper = ServiceProxyWrapper(name=self.name,
                                              service_class=self.service_class_mock)
        arg = MagicMock()
        with self.assertRaises(TypeError):
            service_wrapper(arg)
            assert rospy_mock.ServiceProxy.return_value.call_count == 1
