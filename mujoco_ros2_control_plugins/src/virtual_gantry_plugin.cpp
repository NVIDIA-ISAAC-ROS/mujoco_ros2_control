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

#include <algorithm>
#include <cmath>
#include <vector>

#include <mujoco/mujoco.h>
#include <pluginlib/class_list_macros.hpp>

#include "mujoco_ros2_control_plugins/gantry_keyboard_state.hpp"

namespace mujoco_ros2_control_plugins
{

bool VirtualGantryPlugin::init(
  rclcpp::Node::SharedPtr node, const mjModel * model, mjData * /*data*/)
{
  node_ = node;

  // Parameters live in the parent node's "mujoco_plugins.<plugin_name>.*" namespace,
  // declared via automatically_declare_parameters_from_overrides.
  const std::string prefix = "mujoco_plugins." + node->get_sub_namespace() + ".";
  auto params = node->get_node_parameters_interface();

  auto get_double = [&](const std::string & key, double & out) {
    if (params->has_parameter(prefix + key)) {
      out = params->get_parameter(prefix + key).get_parameter_value().get<double>();
    }
  };
  auto get_string = [&](const std::string & key, std::string & out) {
    if (params->has_parameter(prefix + key)) {
      out = params->get_parameter(prefix + key).get_parameter_value().get<std::string>();
    }
  };

  get_string("body_name", body_name_);
  get_double("kp_pos", kp_pos_);
  get_double("kd_pos", kd_pos_);
  get_double("anchor_z", anchor_z_world_);

  if (params->has_parameter(prefix + "body_offset")) {
    const auto v = params->get_parameter(prefix + "body_offset")
                     .get_parameter_value().get<std::vector<double>>();
    if (v.size() == 3) {
      body_offset_ = {{v[0], v[1], v[2]}};
    }
  }

  body_id_ = mj_name2id(model, mjOBJ_BODY, body_name_.c_str());
  if (body_id_ < 0) {
    RCLCPP_ERROR(node_->get_logger(), "VirtualGantryPlugin: body '%s' not found",
                 body_name_.c_str());
    return false;
  }

  // Resolve optional mocap visual bodies.
  auto resolve_mocap = [&](const std::string & param_key, int & mocap_id_out) {
    std::string name;
    get_string(param_key, name);
    if (name.empty()) {return;}
    const int bid = mj_name2id(model, mjOBJ_BODY, name.c_str());
    if (bid >= 0 && model->body_mocapid[bid] >= 0) {
      mocap_id_out = model->body_mocapid[bid];
      RCLCPP_INFO(node_->get_logger(), "VirtualGantryPlugin: %s '%s' (mocap_id=%d)",
                  param_key.c_str(), name.c_str(), mocap_id_out);
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "VirtualGantryPlugin: '%s' (param %s) not found or not a mocap body",
                  name.c_str(), param_key.c_str());
    }
  };
  resolve_mocap("anchor_visual_body", anchor_mocap_id_);
  resolve_mocap("rope_visual_body", rope_mocap_id_);

  // Find the rope capsule geom for dynamic half-length updates.
  if (rope_mocap_id_ >= 0) {
    std::string rope_name;
    get_string("rope_visual_body", rope_name);
    if (!rope_name.empty()) {
      const int bid = mj_name2id(model, mjOBJ_BODY, rope_name.c_str());
      if (bid >= 0 && model->body_geomnum[bid] > 0) {
        rope_geom_id_ = model->body_geomadr[bid];
      }
    }
  }

  RCLCPP_INFO(node_->get_logger(),
              "VirtualGantryPlugin: body '%s' (id=%d), anchor_z=%.2f, "
              "offset=[%.3f,%.3f,%.3f], kp=%.0f, kd=%.0f",
              body_name_.c_str(), body_id_, anchor_z_world_,
              body_offset_[0], body_offset_[1], body_offset_[2], kp_pos_, kd_pos_);

  enable_srv_ = node_->create_service<mujoco_ros2_control_msgs::srv::SetGantryEnabled>(
    "set_gantry_enabled",
    [this](
      const mujoco_ros2_control_msgs::srv::SetGantryEnabled::Request::SharedPtr req,
      mujoco_ros2_control_msgs::srv::SetGantryEnabled::Response::SharedPtr resp)
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      const bool was_enabled = enabled_;
      enabled_ = req->enabled;
      if (enabled_ && !was_enabled) {
        spawn_pos_captured_ = false;  // re-anchor above current attach position
      }
      resp->success = true;
      resp->message = enabled_ ? "Gantry enabled" : "Gantry disabled";
      RCLCPP_INFO(node_->get_logger(), "%s", resp->message.c_str());
    });

  // Snapshot the current toggle counter so the first update() doesn't misfire.
  last_toggle_count_ = GantryKeyboardState::get().toggle_counter.load(
    std::memory_order_relaxed);

  return true;
}

void VirtualGantryPlugin::update(const mjModel * model, mjData * data)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  // --- Keyboard toggle ('G' key) -------------------------------------------
  const int tc = GantryKeyboardState::get().toggle_counter.load(
    std::memory_order_relaxed);
  if (tc != last_toggle_count_) {
    last_toggle_count_ = tc;
    enabled_ = !enabled_;
    if (enabled_) {
      spawn_pos_captured_ = false;  // re-anchor above current position on next step
    }
    RCLCPP_INFO(node_->get_logger(), "VirtualGantryPlugin: %s via keyboard",
                enabled_ ? "enabled" : "disabled");
  }

  // --- Rope length adjustment (Shift+scroll) --------------------------------
  const int scroll_ticks =
    GantryKeyboardState::get().rope_scroll_ticks.exchange(0, std::memory_order_relaxed);
  if (scroll_ticks != 0) {
    rope_length_ = std::max(0.1, rope_length_ + scroll_ticks * 0.05);
    RCLCPP_INFO(node_->get_logger(), "VirtualGantryPlugin: rope_length=%.3f m", rope_length_);
  }

  // --- Compute attachment point in world frame ------------------------------
  // attach_pos = body CoM + rotation_matrix * body_offset
  const double * xpos = &data->xpos[body_id_ * 3];
  const double * xmat = &data->xmat[body_id_ * 9];

  double attach_pos[3];
  for (int i = 0; i < 3; ++i) {
    attach_pos[i] = xpos[i]
      + xmat[i * 3 + 0] * body_offset_[0]
      + xmat[i * 3 + 1] * body_offset_[1]
      + xmat[i * 3 + 2] * body_offset_[2];
  }

  // --- Capture anchor on first step after (re-)enable ----------------------
  if (!spawn_pos_captured_) {
    // Anchor XY follows the attachment point; Z is fixed in world frame.
    anchor_pos_ = {{attach_pos[0], attach_pos[1], anchor_z_world_}};
    // Rope is just taut at spawn: length = vertical gap between anchor and attachment.
    rope_length_ = std::abs(anchor_z_world_ - attach_pos[2]);
    spawn_pos_captured_ = true;
    RCLCPP_INFO(node_->get_logger(),
                "VirtualGantryPlugin: anchor at [%.3f, %.3f, %.3f], rope_length=%.3f m",
                anchor_pos_[0], anchor_pos_[1], anchor_pos_[2], rope_length_);
  }

  // --- Anchor visual --------------------------------------------------------
  if (anchor_mocap_id_ >= 0) {
    data->mocap_pos[anchor_mocap_id_ * 3 + 0] = anchor_pos_[0];
    data->mocap_pos[anchor_mocap_id_ * 3 + 1] = anchor_pos_[1];
    data->mocap_pos[anchor_mocap_id_ * 3 + 2] = anchor_pos_[2];
  }

  // --- Rope visual ----------------------------------------------------------
  // Rope vector from anchor to attachment point.
  const double dx = attach_pos[0] - anchor_pos_[0];
  const double dy = attach_pos[1] - anchor_pos_[1];
  const double dz = attach_pos[2] - anchor_pos_[2];
  const double rope_dist = std::sqrt(dx * dx + dy * dy + dz * dz);

  if (rope_mocap_id_ >= 0 && rope_dist > 1e-6) {
    // Midpoint between anchor and attachment.
    data->mocap_pos[rope_mocap_id_ * 3 + 0] = (anchor_pos_[0] + attach_pos[0]) * 0.5;
    data->mocap_pos[rope_mocap_id_ * 3 + 1] = (anchor_pos_[1] + attach_pos[1]) * 0.5;
    data->mocap_pos[rope_mocap_id_ * 3 + 2] = (anchor_pos_[2] + attach_pos[2]) * 0.5;

    // Normalised direction: anchor → attachment.
    const double ndx = dx / rope_dist;
    const double ndy = dy / rope_dist;
    const double ndz = dz / rope_dist;

    // Quaternion aligning capsule Z-axis with the anchor→attachment direction.
    // Rotation axis = cross([0,0,1], [ndx,ndy,ndz]) = [-ndy, ndx, 0].
    const double ax = -ndy, ay = ndx;
    const double axis_len = std::sqrt(ax * ax + ay * ay);
    double qw, qx, qy, qz;
    if (axis_len < 1e-6) {
      // Direction is (anti-)parallel to Z.
      if (ndz > 0.0) {qw = 1.0; qx = 0.0; qy = 0.0; qz = 0.0;}
      else           {qw = 0.0; qx = 1.0; qy = 0.0; qz = 0.0;}
    } else {
      const double clamped = ndz > 1.0 ? 1.0 : ndz < -1.0 ? -1.0 : ndz;
      const double angle = std::acos(clamped);
      const double sa = std::sin(angle * 0.5);
      qw = std::cos(angle * 0.5);
      qx = (ax / axis_len) * sa;
      qy = (ay / axis_len) * sa;
      qz = 0.0;
    }
    data->mocap_quat[rope_mocap_id_ * 4 + 0] = qw;
    data->mocap_quat[rope_mocap_id_ * 4 + 1] = qx;
    data->mocap_quat[rope_mocap_id_ * 4 + 2] = qy;
    data->mocap_quat[rope_mocap_id_ * 4 + 3] = qz;

    // Update capsule half-length to match the current anchor–attachment distance.
    // const_cast is intentional: geom_size is a display property; safe to
    // modify at runtime without affecting physics.
    if (rope_geom_id_ >= 0) {
      const_cast<mjModel *>(model)->geom_size[rope_geom_id_ * 3 + 1] = rope_dist * 0.5;
    }
  }

  // --- Rope constraint force ------------------------------------------------
  // Clear previously applied forces before writing new values.
  for (int i = 0; i < 6; ++i) {
    data->xfrc_applied[body_id_ * 6 + i] = 0.0;
  }

  if (!enabled_ || rope_dist <= rope_length_) {
    return;  // rope slack or gantry disabled — no force applied
  }

  // The rope is taut: apply a radial-only tension force toward the anchor.
  // No tangential component is applied, so the robot swings freely like a pendulum.
  const double rope_dir[3] = {dx / rope_dist, dy / rope_dist, dz / rope_dist};

  // Radial velocity (positive = attachment moving away from anchor).
  const double vx = data->cvel[body_id_ * 6 + 3];
  const double vy = data->cvel[body_id_ * 6 + 4];
  const double vz = data->cvel[body_id_ * 6 + 5];
  const double vel_radial = vx * rope_dir[0] + vy * rope_dir[1] + vz * rope_dir[2];

  // Tension spring + one-sided damping (damp only when rope is extending).
  const double extension = rope_dist - rope_length_;
  const double tension = kp_pos_ * extension + kd_pos_ * std::max(0.0, vel_radial);

  // Force toward anchor (opposite of rope_dir).
  const double Fx = -tension * rope_dir[0];
  const double Fy = -tension * rope_dir[1];
  const double Fz = -tension * rope_dir[2];

  data->xfrc_applied[body_id_ * 6 + 0] = Fx;
  data->xfrc_applied[body_id_ * 6 + 1] = Fy;
  data->xfrc_applied[body_id_ * 6 + 2] = Fz;

  // Torque correction for off-CoM attachment: τ = offset_world × F.
  const double ox = xmat[0] * body_offset_[0] + xmat[1] * body_offset_[1]
                  + xmat[2] * body_offset_[2];
  const double oy = xmat[3] * body_offset_[0] + xmat[4] * body_offset_[1]
                  + xmat[5] * body_offset_[2];
  const double oz = xmat[6] * body_offset_[0] + xmat[7] * body_offset_[1]
                  + xmat[8] * body_offset_[2];

  data->xfrc_applied[body_id_ * 6 + 3] = oy * Fz - oz * Fy;
  data->xfrc_applied[body_id_ * 6 + 4] = oz * Fx - ox * Fz;
  data->xfrc_applied[body_id_ * 6 + 5] = ox * Fy - oy * Fx;
}

void VirtualGantryPlugin::reset()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  // Re-capture anchor on the next update() so the gantry re-anchors above the
  // robot's current position after every simulation reset.
  spawn_pos_captured_ = false;
}

void VirtualGantryPlugin::cleanup()
{
  enable_srv_.reset();
  node_.reset();
}

}  // namespace mujoco_ros2_control_plugins

PLUGINLIB_EXPORT_CLASS(
  mujoco_ros2_control_plugins::VirtualGantryPlugin,
  mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
