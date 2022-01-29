///////////////////////////////////////////////////////////////////////////////////
//   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          //
//                                                                               //
//   Licensed under the Apache License, Version 2.0 (the "License").             //
//   You may not use this file except in compliance with the License.            //
//   You may obtain a copy of the License at                                     //
//                                                                               //
//       http://www.apache.org/licenses/LICENSE-2.0                              //
//                                                                               //
//   Unless required by applicable law or agreed to in writing, software         //
//   distributed under the License is distributed on an "AS IS" BASIS,           //
//   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    //
//   See the License for the specific language governing permissions and         //
//   limitations under the License.                                              //
///////////////////////////////////////////////////////////////////////////////////

#ifndef DEEPSIM_GAZEBO_PLUGIN__GEOMETRY_CONVERTER_HPP_
#define DEEPSIM_GAZEBO_PLUGIN__GEOMETRY_CONVERTER_HPP_

#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Transform.h>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

#include <gazebo/msgs/msgs.hh>

namespace deepsim_gazebo_plugin
{
class GeometryConverter
{
public:
    /// \brief Specialized conversion from a ROS vector message to an Ignition Math vector.
    /// \param[in] msg ROS message to convert.
    /// \return An Ignition Math vector.
    static ignition::math::Vector3d Convert2MathVector3d(const geometry_msgs::Vector3 & msg);

    /// \brief Specialized conversion from a ROS point message to a ROS vector message.
    /// \param[in] in ROS message to convert.
    /// \return ROS geometry_msgs::Vector3
    static geometry_msgs::Vector3 Convert2GeoVector3(const geometry_msgs::Point & in);

    /// \brief Specialized conversion from a ROS point message to an Ignition math vector.
    /// \param[in] in ROS message to convert.
    /// \return An Ignition Math vector.
    static ignition::math::Vector3d Convert2MathVector3d(const geometry_msgs::Point & in);

    /// \brief Specialized conversion from an Ignition Math vector to a ROS message.
    /// \param[in] vec Ignition vector to convert.
    /// \return ROS geometry vector message
    static geometry_msgs::Vector3 Convert2GeoVector3(const ignition::math::Vector3d & vec);

    /// \brief Specialized conversion from an Ignition Math vector to a ROS message.
    /// \param[in] vec Ignition vector to convert.
    /// \return ROS geometry point message
    static geometry_msgs::Point Convert2GeoPoint(const ignition::math::Vector3d & vec);

    /// \brief Specialized conversion from an Ignition Math Quaternion to a ROS message.
    /// \param[in] in Ignition Quaternion to convert.
    /// \return ROS geometry quaternion message
    static geometry_msgs::Quaternion Convert2GeoQuaternion(const ignition::math::Quaterniond & in);

    /// \brief Specialized conversion from an Ignition Math Pose3d to a ROS geometry transform message.
    /// \param[in] in Ignition Pose3d to convert.
    /// \return ROS geometry transform message
    static geometry_msgs::Transform Convert2GeoTransform(const ignition::math::Pose3d & in);

    /// \brief Specialized conversion from an Ignition Math Pose3d to a ROS geometry pose message.
    /// \param[in] in Ignition Pose3d to convert.
    /// \return ROS geometry pose message
    static geometry_msgs::Pose Convert2GeoPose(const ignition::math::Pose3d & in);

    /// \brief Specialized conversion from a ROS quaternion message to ignition quaternion
    /// \param[in] in Input quaternion message
    /// \return Ignition math quaternion with same values as the input message
    static ignition::math::Quaterniond Convert2MathQuaterniond(const geometry_msgs::Quaternion & in);

    /// \brief Specialized conversion from a ROS geometry transform message to an Ignition math pose3d.
    /// \param[in] in ROS message to convert.
    /// \return A Ignition Math pose3d.
    static ignition::math::Pose3d Convert2MathPose3d(const geometry_msgs::Transform & in);

    /// \brief Specialized conversion from a ROS pose message to a ROS geometry transform message.
    /// \param[in] in ROS pose message to convert.
    /// \return A ROS geometry transform message.
    static geometry_msgs::Transform Convert2GeoTransform(const geometry_msgs::Pose & in);

    /// \brief Specialized conversion from a ROS pose message to an Ignition Math pose.
    /// \param[in] in ROS pose message to convert.
    /// \return Ignition Math pose.
    static ignition::math::Pose3d Convert2MathPose3d(const geometry_msgs::Pose & in);

    /// \brief Specialized conversion from a ROS pose message to an Ignition Math pose.
    /// \param[in] in ROS pose message to convert.
    /// \return ROS geometry pose message
    static geometry_msgs::Pose Convert2GeoPose(const gazebo::msgs::Pose & in);

    /// \brief Specialized conversion from a ROS pose message to an Ignition Math pose.
    /// \param[in] in the gazebo msgs::Vector3d to convert.
    /// \return ROS geometry_msgs::Vector3.
    static geometry_msgs::Vector3 Convert2GeoVector3(const gazebo::msgs::Vector3d & in);

    /// \brief Specialized conversion from a ROS pose message to an Ignition Math pose.
    /// \param[in] in ROS gazebo::msgs::Quaternion message to convert.
    /// \return geometry_msgs::Quaternion
    static geometry_msgs::Quaternion Convert2GeoQuaternion(const gazebo::msgs::Quaternion & in);

    /// \brief Specialized conversion from a gazebo vector3d message to an Ignition Math pose.
    /// \param[in] in gazebo::msgs::Vector3d gazebo message to convert
    /// \return ROS geometry_msgs::Point message
    static geometry_msgs::Point Convert2GeoPoint(const gazebo::msgs::Vector3d & in);

    /// \brief Get a geometry_msgs::Vector3 of all ones
    /// \return geometry_msgs::Vector3 of all ones
    static geometry_msgs::Vector3 GeoVector3Ones();
};
}  // namespace deepsim_gazebo_plugin
#endif  // DEEPSIM_GAZEBO_PLUGIN__GEOMETRY_CONVERTER_HPP_
