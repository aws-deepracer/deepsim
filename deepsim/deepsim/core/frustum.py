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
"""A class for frustum."""
import math
from typing import Optional, Union, Tuple, List, Dict
from threading import RLock

from deepsim.core.euler import Euler
from deepsim.core.vector3 import Vector3
from deepsim.core.point import Point
from deepsim.core.pose import Pose
from deepsim.core.plane import Plane
from deepsim.core.ray import Ray
from deepsim.behaviours.transform import Transform


class Frustum:
    def __init__(self,
                 near: float,
                 far: float,
                 horizontal_fov: float,
                 view_ratio: float,
                 transform: Optional[Transform] = None, *,
                 pose: Optional[Pose] = None,
                 pose_offset: Optional[Pose] = None):
        """
        Initialize Frustum class.
        - By default, frustum assumes camera forward is +x axis and up is +z axis.

        Args:
            near (float): near distance from camera position.
            far (float): far distance from camera position.
            horizontal_fov (float): horizontal field of view in radian.
            view_ratio (float): view ratio (height / width)
            transform (Optional[Transform]): transform to be tracked by the frustum.
            pose (Optional[Pose]): pose of the camera.
            pose_offset (Optional[Pose]): pose offset.
        """
        self._near = near
        self._far = far
        self._horizontal_fov = horizontal_fov
        self._view_ratio = view_ratio

        self._transform = transform
        self._transform_lock = RLock()

        self._pose = pose.copy() if pose else Pose()
        self._pose_offset = pose_offset.copy() if pose_offset else Pose()

        self._frustum_lock = RLock()
        self._outdated = False
        self._last_cam_pose = self.adjusted_pose

        # Placeholder
        self._near_plane = None
        self._far_plane = None
        self._left_plane = None
        self._right_plane = None
        self._top_plane = None
        self._bottom_plane = None
        self._near_plane_info = {}
        self._planes = []

    def _set_outdated(self) -> None:
        """
        Set outdated flag when any of the values changed.
        """
        self._outdated = True

    @property
    def near(self) -> float:
        """
        Returns the near distance from the camera position.

        Returns:
            float: the near distance from the camera position.
        """
        return self._near

    @near.setter
    def near(self, value: float) -> None:
        """
        Set new near distance from the camera position.

        Args:
            value (float): new near distance from the camera position.
        """
        with self._frustum_lock:
            self._near = value
            self._set_outdated()

    @property
    def far(self) -> float:
        """
        Returns the far distance from the camera position.

        Returns:
            float: the far distance from the camera position.
        """
        return self._far

    @far.setter
    def far(self, value: float) -> None:
        """
        Set new far distance from the camera position.

        Args:
            value (float): new far distance from the camera position.
        """
        with self._frustum_lock:
            self._far = value
            self._set_outdated()

    @property
    def horizontal_fov(self) -> float:
        """
        Returns the horizontal field of view of the camera.

        Returns:
            float: the horizontal field of view of the camera.
        """
        return self._horizontal_fov

    @horizontal_fov.setter
    def horizontal_fov(self, value: float) -> None:
        """
        Set new horizontal field of view of the camera.

        Args:
            value (float): new horizontal field of view of the camera.
        """
        with self._frustum_lock:
            self._horizontal_fov = value
            self._set_outdated()

    @property
    def view_ratio(self) -> float:
        """
        Returns the view ratio (height / width) of the camera.

        Returns:
            float: the view ratio (height / width) of the camera.
        """
        return self._view_ratio

    @view_ratio.setter
    def view_ratio(self, value: float) -> None:
        """
        Set new view ratio (height / width) of the camera.

        Args:
            value (float): new view ratio (height / width) of the camera.
        """
        with self._frustum_lock:
            self._view_ratio = value
            self._set_outdated()

    def _is_outdated(self) -> bool:
        """
        Returns the flag whether internal data is outdated or not.

        Returns:
            bool: the flag whether internal data is outdated or not.
        """
        with self._frustum_lock:
            return self._outdated or self._last_cam_pose != self.adjusted_pose

    @property
    def pose(self) -> Pose:
        """
        Returns the copy of pose.
        - If tracking transform then it will return transform pose, otherwise will return the copy of internal pose.

        Returns:
            Pose: transform pose if tracking transform otherwise the copy of internal pose.
        """
        transform = self.transform
        if transform:
            return transform.state.pose
        return self._pose.copy()

    @pose.setter
    def pose(self, value: Pose) -> None:
        """
        Set pose with given pose.
        - This won't have any impact during tracking the transform.

        Args:
            value (Pose): new pose
        """
        with self._frustum_lock:
            self._pose = value.copy()
            if not self.transform:
                self._set_outdated()

    @property
    def pose_offset(self) -> Pose:
        """
        Returns a copy of pose offset.

        Returns:
            Pose: a copy of pose offset.
        """
        return self._pose_offset.copy()

    @pose_offset.setter
    def pose_offset(self, value: Pose) -> None:
        """
        Set pose offset with a copy of given pose.

        Args:
            value (Pose): new pose
        """
        with self._frustum_lock:
            self._pose_offset = value.copy()
            self._set_outdated()

    @property
    def adjusted_pose(self) -> Pose:
        """
        Returns the adjusted pose (pose + pose_offset).

        Returns:
            Pose: the adjusted pose (pose + pose_offset).
        """
        return self.pose + self._pose_offset

    @property
    def transform(self) -> Transform:
        """
        Returns the transform tracking.

        Returns:
            Transform: the transform that the collider is tracking.
        """
        with self._transform_lock:
            return self._transform

    @property
    def near_plane(self) -> Plane:
        """
        Returns the near plane of the frustum calculated.

        Returns:
            Plane: the near plane of the frustum calculated.
        """
        with self._frustum_lock:
            self._calculate_frustum_planes()
            return self._near_plane.copy()

    @property
    def far_plane(self) -> Plane:
        """
        Returns the far plane of the frustum calculated.

        Returns:
            Plane: the far plane of the frustum calculated.
        """
        with self._frustum_lock:
            self._calculate_frustum_planes()
            return self._far_plane.copy()

    @property
    def left_plane(self) -> Plane:
        """
        Returns the left plane of the frustum calculated.

        Returns:
            Plane: the left plane of the frustum calculated.
        """
        with self._frustum_lock:
            self._calculate_frustum_planes()
            return self._left_plane.copy()

    @property
    def right_plane(self) -> Plane:
        """
        Returns the right plane of the frustum calculated.

        Returns:
            Plane: the right plane of the frustum calculated.
        """
        self._calculate_frustum_planes()
        return self._right_plane.copy()

    @property
    def top_plane(self) -> Plane:
        """
        Returns the top plane of the frustum calculated.

        Returns:
            Plane: the top plane of the frustum calculated.
        """
        with self._frustum_lock:
            self._calculate_frustum_planes()
            return self._top_plane.copy()

    @property
    def bottom_plane(self) -> Plane:
        """
        Returns the bottom plane of the frustum calculated.

        Returns:
            Plane: the bottom plane of the frustum calculated.
        """
        with self._frustum_lock:
            self._calculate_frustum_planes()
            return self._bottom_plane.copy()

    def attach(self, transform: Transform) -> None:
        """
        Attach the transform to track

        Args:
            transform [Transform]: the transform to be tracked.
        """
        with self._transform_lock, self._frustum_lock:
            self._transform = transform
            self._set_outdated()

    def detach(self) -> None:
        """
        Detach the transform.
        - collider goes back to the internal representation.
        """
        with self._transform_lock, self._frustum_lock:
            try:
                transform = self.transform
                if transform:
                    self._pose = transform.state.pose
            finally:
                self._transform = None
                self._set_outdated()

    @staticmethod
    def project_to_2d(world_point: Union[Vector3, Point],
                      cam_pose: Pose,
                      plane: Plane,
                      plane_center: Union[Vector3, Point],
                      plane_width: float,
                      plane_height: float) -> Tuple[float, float]:
        """
        Project given world point to 2d plane with given plane as a reference 2D plane.

        Args:
            world_point (Union[Vector3, Point]): world point to transform.
            cam_pose (Pose): Camera pose.
            plane (Plane): reference 2D plane
            plane_center (Union[Vector3, Point]): reference plane's center position.
            plane_width (float): width of 2D plane.
            plane_height (float): height of 2D plane.

        Returns:
            Tuple[float, float]: (x, y) position in 2D. (each value is between 0.0 and 1.0)
        """
        world_point = world_point.to_vector() if isinstance(world_point, Point) else world_point
        plane_center = plane_center.to_vector() if isinstance(plane_center, Point) else plane_center

        cam_pos = cam_pose.position.to_vector()
        plane_orientation = cam_pose.orientation
        ray = Ray(origin=world_point,
                  direction=cam_pos - world_point)
        hit = plane.raycast(ray)

        if hit is None:
            return -1.0, -1.0
        else:
            point_on_plane = ray.get_point(hit.entry)

            # Transpose the center back to origin
            point_on_plane_from_origin = point_on_plane - plane_center

            # Reverse the rotation so plane can align back to y-axis
            inverse_cam_quaternion = plane_orientation.inverse()
            point_on_y_axis = point_on_plane_from_origin.rotate(inverse_cam_quaternion)

            # Rotate pitch 90 degree and yaw 90 degree, so plane will align to x and y axis
            # Remember rotation order is roll, pitch, yaw in euler_to_quaternion method
            project_2d_quaternion = Euler(pitch=math.pi / 2.0, yaw=math.pi / 2.0)
            point_on_2d_plane = point_on_y_axis.rotate(project_2d_quaternion)

            # Align plane to origin at x, y = (0, 0)
            point_on_2d_plane = point_on_2d_plane + Vector3(buffer=[plane_width / 2.0, plane_height / 2.0, 0.0])

            # Re-scale x and y space between 0 and 1
            return (point_on_2d_plane[0] / plane_width), (point_on_2d_plane[1] / plane_height)

    @staticmethod
    def calculate_frustum_planes(pose: Pose,
                                 near: float,
                                 far: float,
                                 horizontal_fov: float,
                                 view_ratio: float) -> Tuple[List[Plane], Dict[str, object]]:
        """
        Calculate frustum planes.

        Args:
            pose (Pose): the camera pose.
            near (float): near distance from the camera position.
            far (float): far distance from the camera position.
            horizontal_fov (float): horizontal field of view of the camera.
            view_ratio (float): view ratio (height / width) of the camera.

        Returns:
            Tuple[List[Plane], Dict[str, object]]: frustum planes and near plane information.
                plane order: near, far, left, right, top, bottom.
                near plane information: width, height, position.
        """
        # Get camera position by applying position offset from the car position.
        cam_pose = pose
        cam_pos = cam_pose.position.to_vector()
        cam_orientation = cam_pose.orientation

        cam_forward = Vector3.forward().rotate(cam_orientation)
        cam_up = Vector3.up().rotate(cam_orientation)
        cam_right = Vector3.right().rotate(cam_orientation)

        near_center = cam_pos + cam_forward * near
        far_center = cam_pos + cam_forward * far

        near_width = 2.0 * math.tan(horizontal_fov * 0.5) * near
        near_height = near_width * view_ratio
        far_width = 2.0 * math.tan(horizontal_fov * 0.5) * far
        far_height = far_width * view_ratio

        far_top_left = far_center + cam_up * (far_height * 0.5) - cam_right * (far_width * 0.5)
        far_top_right = far_center + cam_up * (far_height * 0.5) + cam_right * (far_width * 0.5)
        far_bottom_left = far_center - cam_up * (far_height * 0.5) - cam_right * (far_width * 0.5)
        far_bottom_right = far_center - cam_up * (far_height * 0.5) + cam_right * (far_width * 0.5)

        near_top_left = near_center + cam_up * (near_height * 0.5) - cam_right * (near_width * 0.5)
        near_top_right = near_center + cam_up * (near_height * 0.5) + cam_right * (near_width * 0.5)
        near_bottom_left = near_center - cam_up * (near_height * 0.5) - cam_right * (near_width * 0.5)
        near_bottom_right = near_center - cam_up * (near_height * 0.5) + cam_right * (near_width * 0.5)

        # near plane
        p0, p1, p2 = near_bottom_left, near_top_left, near_bottom_right
        near_plane = Plane(points=[p0, p1, p2])

        # far plane
        p0, p1, p2 = far_bottom_right, far_top_right, far_bottom_left
        far_plane = Plane(points=[p0, p1, p2])

        # left plane
        p0, p1, p2 = far_bottom_left, far_top_left, near_bottom_left
        left_plane = Plane(points=[p0, p1, p2])

        # right plane
        p0, p1, p2 = near_bottom_right, near_top_right, far_bottom_right
        right_plane = Plane(points=[p0, p1, p2])

        # top plane
        p0, p1, p2 = near_top_right, near_top_left, far_top_right
        top_plane = Plane(points=[p0, p1, p2])

        # bottom plane
        p0, p1, p2 = near_bottom_right, far_bottom_right, near_bottom_left
        bottom_plane = Plane(points=[p0, p1, p2])

        near_plane_info = {
            "width": near_width,
            "height": near_height,
            "position": near_center
        }

        planes = [near_plane, far_plane,
                  left_plane, right_plane,
                  top_plane, bottom_plane]
        return planes, near_plane_info

    def _calculate_frustum_planes(self) -> None:
        """
        Calculate frustum planes if outdated.
        - calculates new frustum planes if any values or pose are changed.
        """
        if self._is_outdated():
            # Get camera position by applying position offset from the car position.
            cam_pose = self.adjusted_pose
            self._planes, self._near_plane_info = Frustum.calculate_frustum_planes(pose=cam_pose,
                                                                                   near=self.near,
                                                                                   far=self.far,
                                                                                   horizontal_fov=self.horizontal_fov,
                                                                                   view_ratio=self.view_ratio)

            (self._near_plane, self._far_plane,
             self._left_plane, self._right_plane,
             self._top_plane, self._bottom_plane) = self._planes

            self._last_cam_pose = cam_pose
            self._outdated = False

    def contains(self, target: Union[Vector3, Point]) -> bool:
        """
        Check if the frustum contains with target.

        Args:
            target Union['AbstractCollider', Vector3, Point]: target to check whether frustum contains.

        Returns:
            bool: True if frustum contains target otherwise False.
        """
        with self._frustum_lock:
            self._calculate_frustum_planes()
            for plane in self._planes:
                if plane.which_side(target) <= 0.0:
                    return False
            return True

    def __contains__(self, item: Union[Vector3, Point]) -> bool:
        """
        Check if the frustum contains with target.

        Args:
            target Union['AbstractCollider', Vector3, Point]: target to check whether frustum contains.

        Returns:
            bool: True if frustum contains target otherwise False.
        """
        return self.contains(target=item)

    def world_to_viewport_point(self, target: Union[Vector3, Point]) -> Tuple[float, float]:
        """
        Convert target world position to viewport point in 2D.

        Args:
            target (Union[Vector3, Point]): target world point to convert.

        Returns:
            Tuple[float, float]: converted viewport point in x and y.
        """
        with self._frustum_lock:
            self._calculate_frustum_planes()
            pose = self.adjusted_pose
            target = target.to_vector() if isinstance(target, Point) else target
            return Frustum.project_to_2d(world_point=target,
                                         cam_pose=pose,
                                         plane=self.near_plane,
                                         plane_center=self._near_plane_info["position"],
                                         plane_width=self._near_plane_info["width"],
                                         plane_height=self._near_plane_info["height"])

    def viewport_point_to_ray(self, x: float, y: float) -> Ray:
        """
        Convert viewport point to ray.

        Args:
            x (float): x value of the viewport point.
            y (float): y value of the viewport point.

        Returns:
            Ray: ray pointing from camera position to viewport point.
        """
        if 0.0 > x > 1.0:
            raise ValueError("x({}) must be between 0.0 and 1.0.".format(x))
        if 0.0 > y > 1.0:
            raise ValueError("y({}) must be between 0.0 and 1.0.".format(y))

        with self._frustum_lock:
            self._calculate_frustum_planes()

            x = x - 0.5
            y = y - 0.5
            pose = self.adjusted_pose
            cam_pos = pose.position
            cam_orientation = pose.orientation

            near_width = self._near_plane_info["width"]
            near_height = self._near_plane_info["height"]
            near_center = self._near_plane_info["position"]

            width_offset_from_center = near_width * x
            height_offset_from_center = near_height * y

            # near-plane forward is x-axis and near-plane aligned at y-axis originally prior to rotation.
            pos_offset = Vector3(y=-width_offset_from_center, z=height_offset_from_center)
            point_on_near_plane = near_center + pos_offset.rotate(cam_orientation)
            return Ray(origin=cam_pos, direction=point_on_near_plane - cam_pos)
