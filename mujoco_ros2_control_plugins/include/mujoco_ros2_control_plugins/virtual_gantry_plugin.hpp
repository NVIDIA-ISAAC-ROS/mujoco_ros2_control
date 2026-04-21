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
#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mujoco_ros2_control_plugins
{

// Rope-constraint gantry that suspends a humanoid robot from a fixed anchor point.
//
// Physics model: a one-sided cable constraint.  When the distance from the anchor
// to the configured attachment point on the robot exceeds rope_length_, a radial
// tension force is applied toward the anchor.  No lateral force is ever applied,
// so the robot swings freely like a pendulum at all times.
//
// The anchor spawns anchor_height_ metres directly above the attachment point the
// first time update() runs (and after every re-enable).  The rope_length_ defaults
// to anchor_height_ so the rope is just taut at spawn.
//
// 'G' in the MuJoCo viewer toggles the gantry on/off.
// Shift+scroll adjusts rope_length_ at 5 cm per scroll notch.
// The set_gantry_enabled ROS 2 service also enables/disables the gantry.
class VirtualGantryPlugin : public MuJoCoROS2ControlPluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel * model, mjData * data) override;
  void update(const mjModel * model, mjData * data) override;
  void reset() override;
  void cleanup() override;

private:
  rclcpp::Node::SharedPtr node_;

  // Target body (attachment point on the robot).
  std::string body_name_{"torso_link"};
  int body_id_{-1};

  // Offset from the body CoM to the rope attachment point, expressed in the body frame.
  std::array<double, 3> body_offset_{{0.0, 0.0, 0.0}};

  // Rope tension spring/damper gains.
  double kp_pos_{50000.0};
  double kd_pos_{5000.0};

  // Fixed anchor position (world frame); set once on first update after (re-)enable.
  double anchor_height_{2.0};
  std::array<double, 3> anchor_pos_{};

  // Maximum rope length.  Defaults to anchor_height_ so the rope is taut at spawn.
  double rope_length_{2.0};

  bool enabled_{true};
  bool spawn_pos_captured_{false};

  // mocap_id for the anchor sphere visual (-1 = not present in model).
  int anchor_mocap_id_{-1};
  // mocap_id for the rope capsule visual (-1 = not present in model).
  int rope_mocap_id_{-1};
  // geom index of the rope capsule (for dynamic half-length update, -1 = none).
  int rope_geom_id_{-1};

  // Last toggle_counter value seen; used to detect 'G' key edges.
  int last_toggle_count_{0};

  std::mutex state_mutex_;

  rclcpp::Service<mujoco_ros2_control_msgs::srv::SetGantryEnabled>::SharedPtr enable_srv_;
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__VIRTUAL_GANTRY_PLUGIN_HPP_
