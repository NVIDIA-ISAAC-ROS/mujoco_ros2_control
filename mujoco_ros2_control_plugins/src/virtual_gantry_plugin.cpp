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

#include "mujoco_ros2_control_plugins/virtual_gantry_plugin.hpp"

#include <mujoco/mujoco.h>
#include <pluginlib/class_list_macros.hpp>

namespace mujoco_ros2_control_plugins
{

bool VirtualGantryPlugin::init(
  rclcpp::Node::SharedPtr node, const mjModel * model, mjData * data)
{
  node_ = node;

  // Parameters are read from the parent node's "mujoco_plugins.<plugin_name>.*" namespace,
  // which is already declared via automatically_declare_parameters_from_overrides. We access
  // them through the shared NodeParametersInterface to bypass sub-namespace prepending.
  const std::string prefix = "mujoco_plugins." + node->get_sub_namespace() + ".";
  auto params = node->get_node_parameters_interface();

  if (params->has_parameter(prefix + "body_name")) {
    body_name_ = params->get_parameter(prefix + "body_name").get_parameter_value().get<std::string>();
  }
  if (params->has_parameter(prefix + "kp_pos")) {
    kp_pos_ = params->get_parameter(prefix + "kp_pos").get_parameter_value().get<double>();
  }
  if (params->has_parameter(prefix + "kd_pos")) {
    kd_pos_ = params->get_parameter(prefix + "kd_pos").get_parameter_value().get<double>();
  }

  body_id_ = mj_name2id(model, mjOBJ_BODY, body_name_.c_str());
  if (body_id_ < 0) {
    RCLCPP_ERROR(node_->get_logger(), "VirtualGantryPlugin: body '%s' not found in model",
                 body_name_.c_str());
    return false;
  }

  // Capture spawn position as the hold target.
  target_pos_ = {{
    data->xpos[body_id_ * 3 + 0],
    data->xpos[body_id_ * 3 + 1],
    data->xpos[body_id_ * 3 + 2],
  }};

  RCLCPP_INFO(node_->get_logger(),
              "VirtualGantryPlugin: holding '%s' (id=%d) at [%.3f, %.3f, %.3f], "
              "kp=%.1f, kd=%.1f",
              body_name_.c_str(), body_id_,
              target_pos_[0], target_pos_[1], target_pos_[2],
              kp_pos_, kd_pos_);

  enable_srv_ = node_->create_service<mujoco_ros2_control_msgs::srv::SetGantryEnabled>(
    "set_gantry_enabled",
    [this](
      const mujoco_ros2_control_msgs::srv::SetGantryEnabled::Request::SharedPtr req,
      mujoco_ros2_control_msgs::srv::SetGantryEnabled::Response::SharedPtr resp)
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      enabled_ = req->enabled;
      resp->success = true;
      resp->message = enabled_ ? "Gantry enabled" : "Gantry disabled";
      RCLCPP_INFO(node_->get_logger(), "%s", resp->message.c_str());
    });

  target_srv_ = node_->create_service<mujoco_ros2_control_msgs::srv::SetGantryTarget>(
    "set_gantry_target",
    [this](
      const mujoco_ros2_control_msgs::srv::SetGantryTarget::Request::SharedPtr req,
      mujoco_ros2_control_msgs::srv::SetGantryTarget::Response::SharedPtr resp)
    {
      std::lock_guard<std::mutex> lock(target_mutex_);
      target_pos_ = {{req->target_position.x, req->target_position.y, req->target_position.z}};
      resp->success = true;
      resp->message = "Gantry target updated";
      RCLCPP_INFO(node_->get_logger(), "Gantry target set to [%.3f, %.3f, %.3f]",
                  target_pos_[0], target_pos_[1], target_pos_[2]);
    });

  return true;
}

void VirtualGantryPlugin::update(const mjModel * /*model*/, mjData * data)
{
  std::lock_guard<std::mutex> lock(target_mutex_);

  if (!enabled_) {
    // Clear any residual forces when disabled.
    data->xfrc_applied[body_id_ * 6 + 3] = 0.0;
    data->xfrc_applied[body_id_ * 6 + 4] = 0.0;
    data->xfrc_applied[body_id_ * 6 + 5] = 0.0;
    return;
  }

  // Linear velocity of the body (cvel layout: [angular(3), linear(3)] per body).
  const double vx = data->cvel[body_id_ * 6 + 3];
  const double vy = data->cvel[body_id_ * 6 + 4];
  const double vz = data->cvel[body_id_ * 6 + 5];

  // PD forces: spring pulls toward target, damper opposes velocity.
  data->xfrc_applied[body_id_ * 6 + 3] =
    kp_pos_ * (target_pos_[0] - data->xpos[body_id_ * 3 + 0]) - kd_pos_ * vx;
  data->xfrc_applied[body_id_ * 6 + 4] =
    kp_pos_ * (target_pos_[1] - data->xpos[body_id_ * 3 + 1]) - kd_pos_ * vy;
  data->xfrc_applied[body_id_ * 6 + 5] =
    kp_pos_ * (target_pos_[2] - data->xpos[body_id_ * 3 + 2]) - kd_pos_ * vz;
}

void VirtualGantryPlugin::cleanup()
{
  enable_srv_.reset();
  target_srv_.reset();
  node_.reset();
}

}  // namespace mujoco_ros2_control_plugins

PLUGINLIB_EXPORT_CLASS(
  mujoco_ros2_control_plugins::VirtualGantryPlugin,
  mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
