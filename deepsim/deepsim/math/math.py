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
"""A class for math."""
import math
from typing import Tuple, Union, TypeVar
import numpy as np

Point = TypeVar('Point')
Vector3 = TypeVar('Vector3')
Quaternion = TypeVar('Quaternion')
Euler = TypeVar('Euler')


# The order of rotation applied: yaw (z) -> pitch (y) -> roll (x)
def euler_to_quaternion(roll: float = 0, pitch: float = 0, yaw: float = 0) -> Tuple[float, float, float, float]:
    """
    Convert Euler to Quaternion

    Args:
        roll (float): roll angle in radian (x-axis)
        pitch (float): pitch angle in radian (y-axis)
        yaw (float): yaw angle in radian (z-axis)

    Returns:
        Tuple[float, float, float, float]: x, y, z, w
    """
    # Abbreviations for the various angular functions
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    # Quaternion
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return x, y, z, w


def quaternion_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """
    Convert quaternion x, y, z, w to euler angle roll, pitch, yaw

    Args:
        x (float): quaternion x
        y (float): quaternion y
        z (float): quaternion z
        w (float): quaternion w

    Returns:
        Tuple: (roll, pitch, yaw) in radian
    """
    # roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def project_to_2d(point_on_plane: Union[Point, Vector3],
                  plane_center: Union[Point, Vector3],
                  plane_width: float,
                  plane_height: float,
                  plane_quaternion: Quaternion) -> Tuple[float, float]:
    """
    Project the point to 2d.

    Args:
        point_on_plane (Union[Point, Vector3]): the point on plane to project.
        plane_center (Union[Point, Vector3]): plane center
        plane_width (float): width of plane
        plane_height (float): height of plane
        plane_quaternion (Quaternion): plane orientation

    Returns:
        Tuple[float, float]: x and y in 2d space scaled between 0.0 and 1.0.
    """
    from deepsim.math.vector3 import Vector3
    from deepsim.math.quaternion import Quaternion
    from deepsim.math.euler import Euler
    point_on_plane = point_on_plane if isinstance(point_on_plane, Vector3) else point_on_plane.to_vector()
    plane_center = plane_center if isinstance(plane_center, Vector3) else plane_center.to_vector()

    # Transpose the center back to origin
    point_on_plane_from_origin = point_on_plane - plane_center

    # Reverse the rotation so plane can align back to y-axis
    inverse_cam_quaternion = plane_quaternion.inverse()
    point_on_y_axis = point_on_plane_from_origin.rotate(inverse_cam_quaternion)

    # Rotate pitch 90 degree and yaw 90 degree, so plane will align to x and y axis
    # Remember rotation order is roll, pitch, yaw in euler_to_quaternion method
    project_2d_quaternion = Quaternion.from_euler(Euler(pitch=np.pi / 2.0, yaw=np.pi / 2.0))
    point_on_2d_plane = point_on_y_axis.rotate(project_2d_quaternion)

    # Align plane to origin at x, y = (0, 0)
    point_on_2d_plane = point_on_2d_plane + Vector3.from_list([plane_width / 2.0, plane_height / 2.0, 0.0])

    # Re-scale x and y space between 0 and 1
    return (point_on_2d_plane[0] / plane_width), (point_on_2d_plane[1] / plane_height)


def lerp(a: float, b: float, t: float) -> float:
    """
    Linear Interpolation

    Args:
        a (float): start value
        b (float): end value
        t (float): fraction

    Returns:
        float: interpolated value
    """
    t = np.clip(t, 0.0, 1.0).item()
    return a + t * (b - a)


def lerp_angle_rad(a: float, b: float, t: float) -> float:
    """
    Angular Linear Interpolation in radian.

    Args:
        a (float): current angle value in radian
        b (float): target angle value in radian
        t (float): fraction

    Returns:
        float: interpolated angle value in radian
    """
    t = np.clip(t, 0.0, 1.0).item()
    max_ang = 2.0 * math.pi
    diff = b - a
    da = np.clip(diff - diff // max_ang * max_ang, 0.0, max_ang)
    return lerp(a, a + ((da - max_ang) if da > math.pi else da), t)


def cross(v1: Vector3, v2: Vector3) -> Vector3:
    """
    Cross product of two vectors

    Args:
        v1 (Vector3): first vector
        v2 (Vector3): second vector

    Returns:
        Vector3: Cross product of two vectors
    """
    return v1.cross(v2)


def dot(v1: Union[Vector3, Quaternion], v2: Union[Vector3, Quaternion]) -> float:
    """
    Dot product of two vectors or two quaternions

    Args:
        v1 (Union[Vector3, Quaternion]): first vector or quaternion
        v2 (Union[Vector3, Quaternion]): second vector or quaternion

    Returns:
        float: Dot product of two vectors or two quaternions
    """
    return v1.dot(v2)


def magnitude(item: Union[Vector3, Quaternion]) -> float:
    """
    Returns the magnitude of given vector or quaternion.

    Args:
        item (Union[Vector3, Quaternion]): vector or quaternion to find the magnitude.

    Returns:
        float: the magnitude of given vector or quaternion.
    """
    return math.sqrt(item.dot(item))


def sqr_magnitude(item: Union[Vector3, Quaternion]) -> float:
    """
    Returns the squared magnitude of given vector or quaternion.

    Args:
        item (Union[Vector3, Quaternion]): vector or quaternion to find the squared magnitude.

    Returns:
        float: the squared magnitude of given vector or quaternion.
    """
    return item.dot(item)


def unit(item: Union[Vector3, Quaternion]) -> Union[Vector3, Quaternion]:
    """
    Returns a unit vector or quaternion in the direction of v

    Args:
        item (Union[Vector3, Quaternion]): vector or quaternion to find the unit.

    Returns:
        Union[Vector3, Quaternion]: A unit vector or quaternion in the direction of item
    """
    return item / magnitude(item)


def distance(a: Union[Point, Vector3], b: Union[Point, Vector3]) -> float:
    """
    Returns the distance between v1 and v2.
    - (a - b).magnitude

    Args:
        a (Vector3): vector a
        b (Vector3): vector b

    Returns:
        float: (a - b).manitude
    """
    from deepsim.math.vector3 import Vector3
    a = a if isinstance(a, Vector3) else a.to_vector()
    b = b if isinstance(b, Vector3) else b.to_vector()
    diff = a - b
    return diff.magnitude
