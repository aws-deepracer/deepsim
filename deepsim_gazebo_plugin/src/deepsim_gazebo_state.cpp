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

#include "deepsim_gazebo_plugin/deepsim_gazebo_state.hpp"

#include <vector>

#include "deepsim_gazebo_plugin/converters/geometry_converter.hpp"

namespace deepsim_gazebo_plugin
{
/// Helper function to get the model state.
/// \param[in] _entity gazebo::physics::ModelPtr
/// \param[in] _reference_frame gazebo::physics::EntityPtr
/// \return gazebo_msgs::msg::ModelState
///         The entity state got from gazebo
gazebo_msgs::ModelState DeepsimGazeboState::GetModelState(
  const gazebo::physics::ModelPtr _model,
  const gazebo::physics::EntityPtr _reference_frame)
{
  gazebo_msgs::ModelState model_state;

  auto entity_pose = _model->WorldPose();
  auto entity_lin_vel = _model->WorldLinearVel();
  auto entity_ang_vel = _model->WorldAngularVel();

  if (_reference_frame)
  {
    auto frame_pose = _reference_frame->WorldPose();
    auto frame_lin_vel = _reference_frame->WorldLinearVel();
    auto frame_ang_vel = _reference_frame->WorldAngularVel();

    entity_pose = entity_pose - frame_pose;

    // convert to relative
    entity_lin_vel = frame_pose.Rot().RotateVectorReverse(entity_lin_vel - frame_lin_vel);
    entity_ang_vel = frame_pose.Rot().RotateVectorReverse(entity_ang_vel - frame_ang_vel);
  }

  model_state.model_name = _model->GetScopedName();
  model_state.pose.position = deepsim_gazebo_plugin::GeometryConverter::Convert2GeoPoint(
    entity_pose.Pos());
  model_state.pose.orientation = deepsim_gazebo_plugin::GeometryConverter::Convert2GeoQuaternion(
    entity_pose.Rot());

  model_state.twist.linear = deepsim_gazebo_plugin::GeometryConverter::Convert2GeoVector3(entity_lin_vel);
  model_state.twist.angular = deepsim_gazebo_plugin::GeometryConverter::Convert2GeoVector3(entity_ang_vel);
  return model_state;
}

/// Helper function to get the link state.
/// \param[in] _entity gazebo::physics::LinkPtr
/// \param[in] _reference_frame gazebo::physics::EntityPtr
/// \return gazebo_msgs::msg::LinkState
///         The entity state got from gazebo
gazebo_msgs::LinkState DeepsimGazeboState::GetLinkState(
  const gazebo::physics::LinkPtr _link,
  const gazebo::physics::EntityPtr _reference_frame)
{
  gazebo_msgs::LinkState link_state;

  auto entity_pose = _link->WorldPose();
  auto entity_lin_vel = _link->WorldLinearVel();
  auto entity_ang_vel = _link->WorldAngularVel();

  if (_reference_frame)
  {
    auto frame_pose = _reference_frame->WorldPose();
    auto frame_lin_vel = _reference_frame->WorldLinearVel();
    auto frame_ang_vel = _reference_frame->WorldAngularVel();

    entity_pose = entity_pose - frame_pose;

    // convert to relative
    entity_lin_vel = frame_pose.Rot().RotateVectorReverse(entity_lin_vel - frame_lin_vel);
    entity_ang_vel = frame_pose.Rot().RotateVectorReverse(entity_ang_vel - frame_ang_vel);
  }

  link_state.link_name = _link->GetScopedName();
  link_state.pose.position = deepsim_gazebo_plugin::GeometryConverter::Convert2GeoPoint(
    entity_pose.Pos());
  link_state.pose.orientation = deepsim_gazebo_plugin::GeometryConverter::Convert2GeoQuaternion(
    entity_pose.Rot());

  link_state.twist.linear = deepsim_gazebo_plugin::GeometryConverter::Convert2GeoVector3(entity_lin_vel);
  link_state.twist.angular = deepsim_gazebo_plugin::GeometryConverter::Convert2GeoVector3(entity_ang_vel);
  return link_state;
}

/// Helper function get information from the input entity state.
/// \param[in] _input_state gazebo_msgs::ModelState &
/// \param[in] _reference_frame gazebo::physics::EntityPtr
/// \return EntityStateInfo
EntityStateInfo DeepsimGazeboState::GetInputEntityStateInfo(
  const gazebo_msgs::ModelState & _input_state,
  const gazebo::physics::EntityPtr _reference_frame)
{
  EntityStateInfo entity_state_info;
  auto entity_pos = deepsim_gazebo_plugin::GeometryConverter::Convert2MathVector3d(
    _input_state.pose.position);
  auto entity_rot = deepsim_gazebo_plugin::GeometryConverter::Convert2MathQuaterniond(
    _input_state.pose.orientation);

  // Eliminate invalid rotation (0, 0, 0, 0)
  entity_rot.Normalize();

  ignition::math::Pose3d entity_pose(entity_pos, entity_rot);
  auto entity_lin_vel = deepsim_gazebo_plugin::GeometryConverter::Convert2MathVector3d(
    _input_state.twist.linear);
  auto entity_ang_vel = deepsim_gazebo_plugin::GeometryConverter::Convert2MathVector3d(
    _input_state.twist.angular);

  // Frame of reference
  if (_reference_frame)
  {
    auto frame_pose = _reference_frame->WorldPose();
    entity_pose = entity_pose + frame_pose;

    // Velocities should be commanded in the requested reference
    // frame, so we need to translate them to the world frame
    entity_lin_vel = frame_pose.Rot().RotateVector(entity_lin_vel);
    entity_ang_vel = frame_pose.Rot().RotateVector(entity_ang_vel);
  }
  entity_state_info.entity_pose = entity_pose;
  entity_state_info.entity_lin_vel = entity_lin_vel;
  entity_state_info.entity_ang_vel = entity_ang_vel;
  return entity_state_info;
}

/// Helper function get information from the input entity state.
/// \param[in] _input_state gazebo_msgs::msg::LinkState &
/// \param[in] _reference_frame gazebo::physics::EntityPtr
/// \return EntityStateInfo
EntityStateInfo DeepsimGazeboState::GetInputEntityStateInfo(
  const gazebo_msgs::LinkState & _input_state,
  const gazebo::physics::EntityPtr _reference_frame)
{
  EntityStateInfo entity_state_info;
  auto entity_pos = deepsim_gazebo_plugin::GeometryConverter::Convert2MathVector3d(
    _input_state.pose.position);
  auto entity_rot = deepsim_gazebo_plugin::GeometryConverter::Convert2MathQuaterniond(
    _input_state.pose.orientation);

  // Eliminate invalid rotation (0, 0, 0, 0)
  entity_rot.Normalize();

  ignition::math::Pose3d entity_pose(entity_pos, entity_rot);
  auto entity_lin_vel = deepsim_gazebo_plugin::GeometryConverter::Convert2MathVector3d(
    _input_state.twist.linear);
  auto entity_ang_vel = deepsim_gazebo_plugin::GeometryConverter::Convert2MathVector3d(
    _input_state.twist.angular);

  // Frame of reference
  if (_reference_frame)
  {
    auto frame_pose = _reference_frame->WorldPose();
    entity_pose = entity_pose + frame_pose;

    // Velocities should be commanded in the requested reference
    // frame, so we need to translate them to the world frame
    entity_lin_vel = frame_pose.Rot().RotateVector(entity_lin_vel);
    entity_ang_vel = frame_pose.Rot().RotateVector(entity_ang_vel);
  }
  entity_state_info.entity_pose = entity_pose;
  entity_state_info.entity_lin_vel = entity_lin_vel;
  entity_state_info.entity_ang_vel = entity_ang_vel;
  return entity_state_info;
}

/// Helper function to set the entity with new velocity.
/// \param[in] _input_states_info EntityStateInfo
/// \param[in] _target_entity gazebo::physics::ModelPtr
/// \return bool whether or not the set operation was successful.
bool DeepsimGazeboState::SetVelocity(
  const EntityStateInfo _input_states_info,
  const gazebo::physics::ModelPtr _model)
{
  // Set Velocity
  _model->SetLinearVel(_input_states_info.entity_lin_vel);
  _model->SetAngularVel(_input_states_info.entity_ang_vel);
  return true;
}

/// Helper function to set the entity with new velocity.
/// \param[in] _input_states_info EntityStateInfo
/// \param[in] _target_entity gazebo::physics::ModelPtr
/// \return bool whether or not the set operation was successful.
bool DeepsimGazeboState::SetVelocity(
  const EntityStateInfo _input_states_info,
  const gazebo::physics::LinkPtr _link)
{
  // Set Velocity
  _link->SetLinearVel(_input_states_info.entity_lin_vel);
  _link->SetAngularVel(_input_states_info.entity_ang_vel);
  return true;
}

/// Helper function to set the entity state with new poses.
/// Contains a critical section that pauses and unpauses world physics.
/// \param[in] _input_states_info std::vector<EntityStateInfo>
/// \param[in] _target_entities std::vector<gazebo::physics::ModelPtr>
/// \param[in] _world gazebo::physics::WorldPtr
/// \return bool whether or not the set poses operation was successful.
bool DeepsimGazeboState::SetPoses(
  const std::vector<EntityStateInfo> &_input_states_info,
  const std::vector<gazebo::physics::ModelPtr> &_target_entities,
  const gazebo::physics::WorldPtr _world)
{
  if ( _input_states_info.size() != _target_entities.size() )
  {
    return false;
  }
  // Lock the critical section that pauses and unpause world physics.
  std::lock_guard<std::mutex> lock(world_physics_lock_);
  std::vector<EntityStateInfo>::const_iterator input_state_info_iter =
    _input_states_info.begin();
  std::vector<gazebo::physics::ModelPtr>::const_iterator target_entity_iter =
    _target_entities.begin();

  bool is_paused = _world->IsPaused();
  _world->SetPaused(true);

  for (; input_state_info_iter != _input_states_info.end();
    ++input_state_info_iter, ++target_entity_iter)
  {
    EntityStateInfo entity_state_info = *input_state_info_iter;
    gazebo::physics::ModelPtr target_entity_ptr = *target_entity_iter;
    // Set pose
    target_entity_ptr->SetWorldPose(entity_state_info.entity_pose);
  }
  _world->SetPaused(is_paused);
  return true;
}

/// Helper function to set the link state with new poses.
/// Contains a critical section that pauses and unpauses world physics.
/// \param[in] _input_states_info std::vector<EntityStateInfo>
/// \param[in] _target_entities std::vector<gazebo::physics::LinkPtr>
/// \param[in] _world gazebo::physics::WorldPtr
/// \return bool whether or not the set poses operation was successful.
bool DeepsimGazeboState::SetPoses(
  const std::vector<EntityStateInfo> &_input_states_info,
  const std::vector<gazebo::physics::LinkPtr> &_target_entities,
  const gazebo::physics::WorldPtr _world)
{
  if ( _input_states_info.size() != _target_entities.size() )
  {
    return false;
  }
  // Lock the critical section that pauses and unpause world physics.
  std::lock_guard<std::mutex> lock(world_physics_lock_);

  std::vector<EntityStateInfo>::const_iterator input_state_info_iter =
    _input_states_info.begin();
  std::vector<gazebo::physics::LinkPtr>::const_iterator target_entity_iter =
    _target_entities.begin();

  bool is_paused = _world->IsPaused();
  _world->SetPaused(true);

  for (; input_state_info_iter != _input_states_info.end();
    ++input_state_info_iter, ++target_entity_iter)
  {
    EntityStateInfo entity_state_info = *input_state_info_iter;
    gazebo::physics::LinkPtr target_entity_ptr = *target_entity_iter;
    // Set pose
    target_entity_ptr->SetWorldPose(entity_state_info.entity_pose);
  }
  _world->SetPaused(is_paused);
  return true;
}
}  // namespace deepsim_gazebo_plugin
