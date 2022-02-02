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
""" This ensures modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
"""
import os
from setuptools import find_packages
from distutils.core import setup


def read(fname):
    """
    Args:
        fname (str): dir path to read
    """
    with open(os.path.join(os.path.dirname(__file__), fname)) as f:
        return f.read()


def read_version():
    return read("VERSION").strip()


package_name = "deepsim-btree"

# Declare minimal set for installation
required_packages = [
    "setuptools",
    "flake8>=3.5,<4.0.0",
    "pytest-flake8==1.0.7",
    "pytest-pep257==0.0.5",
    "pytest-timeout==1.4.2",
]

setup_args = {
    "name": package_name,
    "version": read_version(),
    "packages": find_packages(where=".", exclude="test"),
    "package_dir": {"": "."},
    "description": "Behaviour Tree library for DeepSim framework.",
    "long_description": read("README.md"),
    "author": "Amazon Web Services",
    "url": "https://github.com/aws-deepracer/deepsim",
    "license": "Apache License 2.0",
    "keywords": "ML RL Amazon AWS AI DeepSim",
    "classifiers": [
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Natural Language :: English",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
    ],
    "install_requires": required_packages,
}

try:
    from catkin_pkg.python_setup import generate_distutils_setup
    setup_args["name"] = "deepsim_btree"
    PACKAGE_IMPORT = generate_distutils_setup(**setup_args)

    setup(**PACKAGE_IMPORT)
except ImportError:
    setup(**setup_args)
