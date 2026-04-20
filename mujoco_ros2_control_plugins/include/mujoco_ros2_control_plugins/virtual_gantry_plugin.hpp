// Copyright 2026 NVIDIA CORPORATION & AFFILIATES
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__VIRTUAL_GANTRY_PLUGIN_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__VIRTUAL_GANTRY_PLUGIN_HPP_

#include <array>
#include <mutex>
#include <string>

#include "mujoco_ros2_control_msgs/srv/set_gantry_enabled.hpp"
#include "mujoco_ros2_control_msgs/srv/set_gantry_target.hpp"
#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mujoco_ros2_control_plugins
{

// PD spring-damper that holds a body at a fixed world-frame XYZ position.
// Intended as a safety net during policy startup: keeps a humanoid robot upright
// while joints are uncontrolled, without interfering once the policy is active.
// Enable/disable and retarget at runtime via the set_gantry_enabled /
// set_gantry_target ROS 2 services. The hold position is captured automatically
// from the body's world-frame coordinates on the first simulation step.
class VirtualGantryPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel * model, mjData * /*data*/) override;
  void update(const mjModel * model, mjData * data) override;
  void reset() override;
  void cleanup() override;

private:
  rclcpp::Node::SharedPtr node_;

  std::string body_name_{"torso_link"};
  int body_id_{-1};

  double kp_pos_{10000.0};
  double kd_pos_{3000.0};

  std::array<double, 3> target_pos_{};
  std::mutex target_mutex_;

  bool enabled_{true};
  bool spawn_pos_captured_{false};

  rclcpp::Service<mujoco_ros2_control_msgs::srv::SetGantryEnabled>::SharedPtr enable_srv_;
  rclcpp::Service<mujoco_ros2_control_msgs::srv::SetGantryTarget>::SharedPtr target_srv_;
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__VIRTUAL_GANTRY_PLUGIN_HPP_
