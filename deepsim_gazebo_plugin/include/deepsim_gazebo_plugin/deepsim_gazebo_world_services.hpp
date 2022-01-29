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

#ifndef DEEPSIM_GAZEBO_PLUGIN__DEEPSIM_GAZEBO_WORLD_SERVICES_HPP_
#define DEEPSIM_GAZEBO_PLUGIN__DEEPSIM_GAZEBO_WORLD_SERVICES_HPP_

#include <memory>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Light.hh>
#include <std_srvs/Empty.h>

#include "deepsim_msgs/GetLightNames.h"


namespace deepsim_gazebo_plugin
{

class DeepsimGazeboWorldServicesPrivate;

class DeepsimGazeboWorldServices
{
public:
  /// Constructor
  explicit DeepsimGazeboWorldServices(
    std::mutex & _lock,
    ros::CallbackQueue & ros_callback_queue_);

  /// Destructor
  virtual ~DeepsimGazeboWorldServices();

  /// \brief Create world related services.
  /// \param[in] _ros_node gazebo_ros::Node::SharedPtr
  /// The ros node pointer for logging and beyond.
  /// \param[in] _world gazebo_ros::Node::SharedPtr
  /// The gazebo world pointer to fetch additional entities.
  void CreateServices(
    const boost::shared_ptr<ros::NodeHandle> _ros_node,
    const gazebo::physics::WorldPtr _world);

private:
  std::unique_ptr<DeepsimGazeboWorldServicesPrivate> impl_;
};
}  // namespace deepsim_gazebo_plugin
#endif  // DEEPSIM_GAZEBO_PLUGIN__DEEPSIM_GAZEBO_WORLD_SERVICES_HPP_
