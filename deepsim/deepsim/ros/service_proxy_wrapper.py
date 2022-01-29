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
"""A class for ROS service proxy wrapper."""
from typing import Any
import rospy


ROS_SERVICE_ERROR_MSG_FORMAT = "ROS Service call failed, Re-try count: {0}/{1}: {2}"


class ServiceProxyWrapper(object):
    """This class wraps rospy's ServiceProxy method so that we can wait
       5 minutes if a service throws an exception. This is required to prevent
       our metrics from being flooded since an exception is thrown by service
       calls when the cancel simulation API is called.
    """
    def __init__(self, name: str, service_class: object, persistent: bool = False, headers: dict = None,
                 should_wait_for_service: bool = True, max_retry_attempts: int = 5) -> None:
        """
        Initialize ServiceProxyWrapper

        Args:
            name (str): Name of the service to create a client for
            service_class (object): The object type for making a service request
            persistent (bool): flag to whether keep the connection open or not
            headers (dict): (optional) arbitrary headers
            should_wait_for_service (bool): The flag whether wrapper should wait for service or not.
            max_retry_attempts (int): maximum number of retry
        """
        if should_wait_for_service:
            rospy.wait_for_service(name)
        self._client = rospy.ServiceProxy(name=name,
                                          service_class=service_class,
                                          persistent=persistent,
                                          headers=headers)
        self._max_retry_attempts = max_retry_attempts

    @property
    def max_retry_attempts(self) -> int:
        """
        Returns the number of max retry attempts

        Returns:
            int: the number of max retry attempts
        """
        return self._max_retry_attempts

    @max_retry_attempts.setter
    def max_retry_attempts(self, value: int) -> None:
        """
        Sets the number of max retry attempts.

        Args:
            value (int): the number of max retry attempts.
        """
        self._max_retry_attempts = value

    def __call__(self, *args: Any, **kwarg: Any) -> Any:
        """
        Makes a client call for the stored service

        Args:
            args (Any): Arbitrary arguments to pass into the client call
            kwarg (Any): Arbitrary keyword arguments to pass into the client call.

        Returns:
            Any: the return value(s) from service call.
        """
        try_count = 0
        while True:
            try:
                return self._client(*args, **kwarg)
            except TypeError as err:
                rospy.logerr("[ServiceProxyWrapper] Invalid arguments for client: {}".format(err))
                raise err
            except Exception as ex:
                try_count += 1
                if try_count > self._max_retry_attempts:
                    rospy.logerr("[ServiceProxyWrapper] Unable to call service: {}".format(ex))
                    raise ex
                error_message = ROS_SERVICE_ERROR_MSG_FORMAT.format(str(try_count),
                                                                    str(self._max_retry_attempts),
                                                                    ex)
                rospy.logerr(error_message)
