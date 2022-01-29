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

#ifndef DEEPSIM_GAZEBO_PLUGIN__DEEPSIM_GAZEBO_VISUAL_PUBLISHER_HPP_
#define DEEPSIM_GAZEBO_PLUGIN__DEEPSIM_GAZEBO_VISUAL_PUBLISHER_HPP_

#include <memory>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#define protected public
#include <gazebo/physics/physics.hh>
#undef protected
#include <gazebo/transport/transport.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Events.hh>

#include "deepsim_msgs/Visual.h"
#include "deepsim_msgs/Visuals.h"

#include "deepsim_gazebo_plugin/converters/visual_converter.hpp"

namespace deepsim_gazebo_plugin
{

class DeepsimGazeboVisualPublisherPrivate;

class DeepsimGazeboVisualPublisher
{
public:
  /// \brief Contructor that binds the callback queue
  /// to protect variables accessed on callbacks.
  /// \param[in] _ros_callback_queue ros::CallbackQueue call back queue for gz services
  explicit DeepsimGazeboVisualPublisher(
    ros::CallbackQueue & ros_callback_queue_);

  /// Destructor
  virtual ~DeepsimGazeboVisualPublisher();

  /// \brief Create visual related publisher.
  /// \param[in] _gz_node gazebo::transport::NodePtr
  /// The gazebo node pointer for gazebo transport.
  /// \param[in] _ros_node gazebo_ros::Node::SharedPtr
  /// The ros node pointer for logging and beyond.
  /// \param[in] _world gazebo_ros::Node::SharedPtr
  /// The gazebo world pointer to fetch additional entities.
  void CreatePublisher(
    const gazebo::transport::NodePtr _gz_node,
    const boost::shared_ptr<ros::NodeHandle> _ros_node,
    const gazebo::physics::WorldPtr _world);

  /// \brief Function to publish visual message.
  /// \param[in] _link gazebo::physics::LinkPtr
  /// \param[in] _visual_msg gazebo::msgs::Visual
  /// \param[in] block bool if this call is blocking
  void PublishVisualMsg(
    gazebo::physics::LinkPtr &_link,
    const gazebo::msgs::Visual &_visual_msg,
    const bool block);

private:
  std::unique_ptr<DeepsimGazeboVisualPublisherPrivate> impl_;
};
}  // namespace deepsim_gazebo_plugin
#endif  // DEEPSIM_GAZEBO_PLUGIN__DEEPSIM_GAZEBO_VISUAL_PUBLISHER_HPP_
