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

#ifndef DEEPSIM_GAZEBO_PLUGIN__DEEPSIM_GAZEBO_STATE_SERVICES_HPP_
#define DEEPSIM_GAZEBO_PLUGIN__DEEPSIM_GAZEBO_STATE_SERVICES_HPP_

#include <memory>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/LinkState.h>

#include "deepsim_msgs/GetModelStates.h"
#include "deepsim_msgs/GetLinkStates.h"
#include "deepsim_msgs/GetAllModelStates.h"
#include "deepsim_msgs/GetAllLinkStates.h"
#include "deepsim_msgs/SetModelStates.h"
#include "deepsim_msgs/SetLinkStates.h"

namespace deepsim_gazebo_plugin
{

class DeepsimGazeboStateServicesPrivate;

class DeepsimGazeboStateServices
{
public:
  /// Constructor
  explicit DeepsimGazeboStateServices(
    std::mutex & _lock,
    ros::CallbackQueue & ros_callback_queue_);

  /// Destructor
  virtual ~DeepsimGazeboStateServices();

  /// \brief Create world related services.
  /// \param[in] _ros_node gazebo_ros::Node::SharedPtr
  /// The ros node pointer for logging and beyond.
  /// \param[in] _world gazebo_ros::Node::SharedPtr
  /// The gazebo world pointer to fetch additional entities.
  void CreateServices(
    const boost::shared_ptr<ros::NodeHandle> _ros_node,
    const gazebo::physics::WorldPtr _world);

private:
  std::unique_ptr<DeepsimGazeboStateServicesPrivate> impl_;
};
}  // namespace deepsim_gazebo_plugin
#endif  // DEEPSIM_GAZEBO_PLUGIN__DEEPSIM_GAZEBO_STATE_SERVICES_HPP_
