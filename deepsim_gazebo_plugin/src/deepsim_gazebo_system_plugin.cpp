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

#include "deepsim_gazebo_plugin/deepsim_gazebo_world_services.hpp"
#include "deepsim_gazebo_plugin/deepsim_gazebo_state_services.hpp"
#include "deepsim_gazebo_plugin/deepsim_gazebo_state.hpp"
#include "deepsim_gazebo_plugin/deepsim_gazebo_visual_services.hpp"
#include "deepsim_gazebo_plugin/deepsim_gazebo_visual_publisher.hpp"

#include "deepsim_gazebo_plugin/deepsim_gazebo_system_plugin.hpp"

#include <memory>
#include <string>

namespace gazebo
{

class DeepSimGazeboSystemPluginPrivate
{
public:
  /// \brief Protect variables accessed on callbacks.
  std::mutex world_physics_lock_;

  /// \brief Protect variables accessed on callbacks.
  std::mutex visuals_lock_;

  /// \brief The call back queue for ros services
  ros::CallbackQueue ros_callback_queue_;

  /// \brief flag whether plugin is loaded or not
  bool is_plugin_loaded_;

  /// \brief flag whether SigInt received or not
  bool should_stop_;

  /// \brief flag whether world is created or not
  bool is_world_created_;

  /// World services
  deepsim_gazebo_plugin::DeepsimGazeboWorldServices world_services_;

  // State services
  deepsim_gazebo_plugin::DeepsimGazeboStateServices state_services_;

  // State
  deepsim_gazebo_plugin::DeepsimGazeboState gazebo_state_;

  // Visual services
  deepsim_gazebo_plugin::DeepsimGazeboVisualServices visual_services_;

  DeepSimGazeboSystemPluginPrivate()
  : is_plugin_loaded_(false),
    is_world_created_(false),
    should_stop_(false),
    world_services_(
      std::ref(world_physics_lock_),
      std::ref(ros_callback_queue_)),
    state_services_(
      std::ref(world_physics_lock_),
      std::ref(ros_callback_queue_)),
    gazebo_state_(
      std::ref(world_physics_lock_)),
    visual_services_(
      std::ref(visuals_lock_),
      std::ref(ros_callback_queue_)) {}

  /// Callback when SIGINT is received.
  void OnSigInt();

  /// Callback when resetting the gazebo threads.
  /// Updates the ros callback queue.
  void UpdateRosCallbackQueue();

  /// Callback when a world is created.
  /// \param[in] _world_name The world's name
  void OnWorldCreated(const std::string & _world_name);

  /// \brief Update period in seconds.
  double update_period_;

  /// \brief World pointer from Gazebo.
  gazebo::physics::WorldPtr world_;

  /// \brief Gazebo node for communication.
  gazebo::transport::NodePtr gz_node_;

  /// \brief Pointer to ROS Node.
  boost::shared_ptr<ros::NodeHandle> ros_node_;

  /// \brief ROS comm
  boost::shared_ptr<ros::AsyncSpinner> async_ros_spinner_;

  /// \brief To be notified once the world is created.
  gazebo::event::ConnectionPtr world_created_event_;

  /// \brief SigInt event connection
  gazebo::event::ConnectionPtr sigint_event_;

  /// \brief The call back queue for gazebo services
  boost::shared_ptr<boost::thread> gazebo_callback_queue_thread_;
};

DeepSimGazeboSystemPlugin::DeepSimGazeboSystemPlugin()
: impl_(std::make_unique<DeepSimGazeboSystemPluginPrivate>())
{
}

DeepSimGazeboSystemPlugin::~DeepSimGazeboSystemPlugin()
{
  ROS_INFO_NAMED("deepsim_gazebo_system_plugin", "[DeepSimGazeboSystemPlugin] Destructor");

  impl_->sigint_event_.reset();
  impl_->world_created_event_.reset();
  // Do nothing if plugin is not loaded
  if (!impl_->is_plugin_loaded_)
  {
    ROS_INFO_NAMED(
      "deepsim_gazebo_system_plugin",
      "[DeepSimGazeboSystemPlugin] Destructor skipped because never loaded");
    return;
  }
  impl_->gazebo_callback_queue_thread_->join();
  impl_->async_ros_spinner_->stop();
  impl_->gz_node_->Fini();
  // Shutdown the ROS node
  impl_->ros_node_->shutdown();
  ROS_INFO_NAMED("deepsim_gazebo_system_plugin", "[DeepSimGazeboSystemPlugin] ROS Node Handle Shutdown");
  ROS_INFO_NAMED("deepsim_gazebo_system_plugin", "[DeepSimGazeboSystemPlugin] Unloaded");
}

void DeepSimGazeboSystemPlugin::Load(int /* argc */, char ** /* argv */)
{
  ROS_INFO_NAMED("deepsim_gazebo_system_plugin", "[DeepSimGazeboSystemPlugin] Load");

  // Get a callback when SIGINT is received
  impl_->sigint_event_ = gazebo::event::Events::ConnectSigInt(
    boost::bind(
      &DeepSimGazeboSystemPluginPrivate::OnSigInt,
      impl_.get()));

  // Get a callback when a world is created
  impl_->world_created_event_ = gazebo::event::Events::ConnectWorldCreated(
    boost::bind(
      &DeepSimGazeboSystemPluginPrivate::OnWorldCreated,
      impl_.get(),
      _1));
  impl_->is_plugin_loaded_ = true;
  ROS_INFO_NAMED(
    "deepsim_gazebo_system_plugin",
    "[DeepSimGazeboSystemPlugin] Loading Finished.");
}

void DeepSimGazeboSystemPluginPrivate::OnSigInt()
{
    ROS_INFO_NAMED("deepsim_gazebo_system_plugin", "[DeepSimGazeboSystemPlugin] SigInt event received");
    should_stop_ = true;
}

void DeepSimGazeboSystemPluginPrivate::OnWorldCreated(const std::string & _world_name)
{
  ROS_INFO_NAMED("deepsim_gazebo_system_plugin", "[DeepSimGazeboSystemPlugin] World Created.");

  world_physics_lock_.lock();
  if (is_world_created_)
  {
    world_physics_lock_.unlock();
    return;
  }

  // set flag to true and load this plugin
  is_world_created_ = true;
  world_physics_lock_.unlock();
  ROS_INFO_NAMED("deepsim_gazebo_system_plugin", "[DeepSimGazeboSystemPlugin] World Created.");

  // check if the ros master is available - required
  // gazebo_ros_api plugin will init the ros,
  // so deepsim gazebo plugin can just wait for ros master to be alive.
  while (!ros::master::check())
  {
      ROS_INFO_NAMED(
        "deepsim_gazebo_system_plugin",
        "[DeepSimGazeboSystemPlugin] No ROS master - start roscore to continue...");
      // wait 0.5 second
      usleep(500*1000);  // can't use ROS Time here b/c node handle is not yet initialized

      if (should_stop_)
      {
          ROS_INFO_NAMED(
            "deepsim_gazebo_system_plugin",
            "[DeepSimGazeboSystemPlugin] Canceled loading by SigInt event");
          return;
      }
  }
  ros_node_.reset(new ros::NodeHandle("~"));

  async_ros_spinner_.reset(new ros::AsyncSpinner(0));  // will use a thread for each CPU core
  async_ros_spinner_->start();

  world_ = gazebo::physics::get_world();
  if (!world_)
  {
    ROS_FATAL_NAMED(
      "deepsim_gazebo_system_plugin",
      "[DeepSimGazeboSystemPlugin] physics::get_world() fails to return world");
    return;
  }

  // Gazebo transport
  gz_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  gz_node_->Init(_world_name);

  // Create Services
  world_services_.CreateServices(ros_node_, world_);
  state_services_.CreateServices(ros_node_, world_);
  visual_services_.CreateServices(gz_node_, ros_node_, world_);

  gazebo_callback_queue_thread_.reset(
    new boost::thread(
      &DeepSimGazeboSystemPluginPrivate::UpdateRosCallbackQueue,
      this));

  ROS_INFO_NAMED(
    "deepsim_gazebo_system_plugin",
    "[DeepSimGazeboSystemPlugin] Init Complete");
}

void DeepSimGazeboSystemPluginPrivate::UpdateRosCallbackQueue()
{
    static const double timeout = 0.001;
    while (ros_node_->ok())
    {
        ros_callback_queue_.callAvailable(ros::WallDuration(timeout));
    }
}

GZ_REGISTER_SYSTEM_PLUGIN(DeepSimGazeboSystemPlugin)

}  // namespace gazebo
