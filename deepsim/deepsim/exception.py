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
"""Classes for exceptions."""


class DeepSimException(Exception):
    """
    UDEException class
    """
    def __init__(self, message="", error_code=500):
        """
        Initialize UDEException

        Args:
            message (str): message
            error_code (int): error code
        """
        self.error_code = error_code
        self.message = message
        super().__init__(self.message)

    def __str__(self):
        return "{} ({})".format(self.message, self.error_code)


class DeepSimError(DeepSimException):
    """
    DeepSimError class
    """
    def __init__(self, message="", error_code=400):
        """
        Initialize DeepSimError

        Args:
            message (str): message
            error_code (int): error code
        """
        super().__init__(error_code=error_code,
                         message=message)


class DeepSimCallbackError(DeepSimException):
    """
    DeepSimCallbackError class
    """
    def __init__(self, message="", error_code=400):
        """
        Initialize DeepSimCallbackError

        Args:
            message (str): message
            error_code (int): error code
        """
        super().__init__(error_code=error_code,
                         message=message)
