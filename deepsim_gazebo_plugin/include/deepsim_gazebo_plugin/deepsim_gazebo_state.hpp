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

#ifndef DEEPSIM_GAZEBO_PLUGIN__DEEPSIM_GAZEBO_STATE_HPP_
#define DEEPSIM_GAZEBO_PLUGIN__DEEPSIM_GAZEBO_STATE_HPP_

#include <vector>

#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>

#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/LinkState.h>

namespace deepsim_gazebo_plugin
{

typedef struct
{
  ignition::math::Pose3d entity_pose;
  ignition::math::Vector3d entity_lin_vel;
  ignition::math::Vector3d entity_ang_vel;
} EntityStateInfo;

class DeepsimGazeboState
{
public:
  /// initialize DeepsimGazeboState object.
  /// \param[in] lock_ std::mutex bind lock reference during initialization
  explicit DeepsimGazeboState(std::mutex & _lock)
  : world_physics_lock_(_lock) {}

  /// Helper function to get the model state.
  /// \param[in] _entity gazebo::physics::ModelPtr
  /// \param[in] _reference_frame gazebo::physics::EntityPtr
  /// \return gazebo_msgs::msg::ModelState
  ///         The entity state got from gazebo.
  gazebo_msgs::ModelState GetModelState(
    const gazebo::physics::ModelPtr _model,
    const gazebo::physics::EntityPtr _reference_frame);

  /// Helper function to get the link state.
  /// \param[in] _entity gazebo::physics::ModelPtr
  /// \param[in] _reference_frame gazebo::physics::EntityPtr
  /// \return gazebo_msgs::msg::LinkState
  ///         The entity state got from gazebo.
  gazebo_msgs::LinkState GetLinkState(
    const gazebo::physics::LinkPtr _link,
    const gazebo::physics::EntityPtr _reference_frame);

  /// Helper function get information from the input entity state.
  /// \param[in] _input_entity_state gazebo_msgs::msg::ModelState &
  /// \param[in] _reference_frame gazebo::physics::EntityPtr
  /// \return EntityStateInfo
  EntityStateInfo GetInputEntityStateInfo(
    const gazebo_msgs::ModelState & _input_state,
    const gazebo::physics::EntityPtr _reference_frame);

  /// Helper function get information from the input entity state.
  /// \param[in] _input_entity_state gazebo_msgs::msg::LinkState &
  /// \param[in] _reference_frame gazebo::physics::EntityPtr
  /// \return EntityStateInfo
  EntityStateInfo GetInputEntityStateInfo(
    const gazebo_msgs::LinkState & _input_state,
    const gazebo::physics::EntityPtr _reference_frame);

  /// Helper function to set the entity with new velocity.
  /// \param[in] _input_states_info EntityStateInfo
  /// \param[in] _target_entity gazebo::physics::ModelPtr
  /// \return bool whether or not the set operation was successful.
  bool SetVelocity(
    const EntityStateInfo _input_states_info,
    const gazebo::physics::ModelPtr _model);

  /// Helper function to set the entity with new velocity.
  /// \param[in] _input_states_info EntityStateInfo
  /// \param[in] _target_entity gazebo::physics::LinkPtr
  /// \return bool whether or not the set operation was successful.
  bool SetVelocity(
    const EntityStateInfo _input_states_info,
    const gazebo::physics::LinkPtr _link);

  /// Helper function to set the entity state with new poses.
  /// Contains a critical section that pauses and unpauses world physics.
  /// \param[in] _input_states_info std::vector<EntityStateInfo>
  /// \param[in] _target_entities std::vector<gazebo::physics::ModelPtr>
  /// \param[in] _world gazebo::physics::WorldPtr
  /// \return bool whether or not the set poses operation was successful.
  bool SetPoses(
    const std::vector<EntityStateInfo> &_input_states_info,
    const std::vector<gazebo::physics::ModelPtr> &_target_entities,
    const gazebo::physics::WorldPtr _world);

  /// Helper function to set the entity state with new poses.
  /// Contains a critical section that pauses and unpauses world physics.
  /// \param[in] _input_states_info std::vector<EntityStateInfo>
  /// \param[in] _target_entities std::vector<gazebo::physics::LinkPtr>
  /// \param[in] _world gazebo::physics::WorldPtr
  /// \return bool whether or not the set poses operation was successful.
  bool SetPoses(
    const std::vector<EntityStateInfo> &_input_states_info,
    const std::vector<gazebo::physics::LinkPtr> &_target_entities,
    const gazebo::physics::WorldPtr _world);

private:
  std::mutex & world_physics_lock_;
};
}  // namespace deepsim_gazebo_plugin
#endif  // DEEPSIM_GAZEBO_PLUGIN__DEEPSIM_GAZEBO_STATE_HPP_
