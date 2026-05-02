// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "mujoco_ros2_control_plugins/virtual_gantry_plugin.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include <mujoco/mujoco.h>
#include <pluginlib/class_list_macros.hpp>

namespace mujoco_ros2_control_plugins
{

bool VirtualGantryPlugin::init(rclcpp::Node::SharedPtr node, const mjModel* model, mjData* /*data*/)
{
  node_ = node;

  // Parameters live in the parent node's "mujoco_plugins.<plugin_name>.*" namespace,
  // declared via automatically_declare_parameters_from_overrides.
  const std::string prefix = "mujoco_plugins." + node->get_sub_namespace() + ".";
  auto params = node->get_node_parameters_interface();

  auto get_double = [&](const std::string& key, double& out) {
    if (params->has_parameter(prefix + key))
    {
      out = params->get_parameter(prefix + key).get_parameter_value().get<double>();
    }
  };
  auto get_string = [&](const std::string& key, std::string& out) {
    if (params->has_parameter(prefix + key))
    {
      out = params->get_parameter(prefix + key).get_parameter_value().get<std::string>();
    }
  };

  get_string("body_name", body_name_);
  get_double("kp_pos", kp_pos_);
  get_double("kd_pos", kd_pos_);
  get_double("anchor_z", anchor_z_world_);

  if (params->has_parameter(prefix + "body_offset"))
  {
    const auto v = params->get_parameter(prefix + "body_offset").get_parameter_value().get<std::vector<double>>();
    if (v.size() == 3)
    {
      body_offset_ = { { v[0], v[1], v[2] } };
    }
  }

  body_id_ = mj_name2id(model, mjOBJ_BODY, body_name_.c_str());
  if (body_id_ < 0)
  {
    RCLCPP_ERROR(node_->get_logger(), "VirtualGantryPlugin: body '%s' not found", body_name_.c_str());
    return false;
  }

  RCLCPP_INFO(node_->get_logger(),
              "VirtualGantryPlugin: body '%s' (id=%d), anchor_z=%.2f, "
              "offset=[%.3f,%.3f,%.3f], kp=%.0f, kd=%.0f",
              body_name_.c_str(), body_id_, anchor_z_world_, body_offset_[0], body_offset_[1], body_offset_[2], kp_pos_,
              kd_pos_);

  enable_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      "set_gantry_enabled",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req, std_srvs::srv::SetBool::Response::SharedPtr resp) {
        std::lock_guard<std::mutex> lock(state_mutex_);
        const bool was_enabled = enabled_;
        enabled_ = req->data;
        if (enabled_ && !was_enabled)
        {
          spawn_pos_captured_ = false;  // re-anchor above current attach position
        }
        resp->success = true;
        resp->message = enabled_ ? "Gantry enabled" : "Gantry disabled";
        RCLCPP_INFO(node_->get_logger(), "%s", resp->message.c_str());
      });

  // Snapshot the current toggle counter so the first update() doesn't misfire.
  last_toggle_count_ = toggle_counter_.load(std::memory_order_relaxed);

  return true;
}

void VirtualGantryPlugin::update(const mjModel* /*model*/, mjData* data)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  // --- Keyboard toggle ('G' key) -------------------------------------------
  const int tc = toggle_counter_.load(std::memory_order_acquire);
  const int delta = tc - last_toggle_count_;
  if (delta != 0)
  {
    last_toggle_count_ = tc;
    if (delta & 1)
    {
      enabled_ = !enabled_;
      if (enabled_)
      {
        spawn_pos_captured_ = false;  // re-anchor above current position on next step
      }
      RCLCPP_INFO(node_->get_logger(), "VirtualGantryPlugin: %s via keyboard", enabled_ ? "enabled" : "disabled");
    }
  }

  // --- Rope length adjustment ('[' / ']' keys) -------------------------------
  const int scroll_ticks = rope_length_ticks_.exchange(0, std::memory_order_acquire);
  if (scroll_ticks != 0)
  {
    if (enabled_)
    {
      rope_length_ = std::max(0.1, rope_length_ + scroll_ticks * 0.005);
      RCLCPP_INFO(node_->get_logger(), "VirtualGantryPlugin: rope_length=%.3f m", rope_length_);
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "VirtualGantryPlugin: gantry disabled — rope length unchanged");
    }
  }

  // --- Compute attachment point in world frame ------------------------------
  // attach_pos = body CoM + rotation_matrix * body_offset
  const double* xpos = &data->xpos[body_id_ * 3];
  const double* xmat = &data->xmat[body_id_ * 9];

  double attach_pos[3];
  for (int i = 0; i < 3; ++i)
  {
    attach_pos[i] = xpos[i] + xmat[i * 3 + 0] * body_offset_[0] + xmat[i * 3 + 1] * body_offset_[1] +
                    xmat[i * 3 + 2] * body_offset_[2];
  }

  // --- Capture anchor on first step after (re-)enable ----------------------
  if (!spawn_pos_captured_)
  {
    // Anchor XY follows the attachment point; Z is fixed in world frame.
    anchor_pos_ = { { attach_pos[0], attach_pos[1], anchor_z_world_ } };
    // Rope is just taut at spawn: length = vertical gap between anchor and attachment.
    rope_length_ = std::abs(anchor_z_world_ - attach_pos[2]);
    spawn_pos_captured_ = true;
    RCLCPP_INFO(node_->get_logger(), "VirtualGantryPlugin: anchor at [%.3f, %.3f, %.3f], rope_length=%.3f m",
                anchor_pos_[0], anchor_pos_[1], anchor_pos_[2], rope_length_);
  }

  // Rope vector from anchor to attachment point.
  const double dx = attach_pos[0] - anchor_pos_[0];
  const double dy = attach_pos[1] - anchor_pos_[1];
  const double dz = attach_pos[2] - anchor_pos_[2];
  const double rope_dist = std::sqrt(dx * dx + dy * dy + dz * dz);

  // --- Rope constraint force ------------------------------------------------
  // Clear previously applied forces before writing new values.
  for (int i = 0; i < 6; ++i)
  {
    data->xfrc_applied[body_id_ * 6 + i] = 0.0;
  }

  if (!enabled_ || rope_dist < 1e-6 || rope_dist <= rope_length_)
  {
    // Rope is slack or gantry disabled: reset FD state so first taut step has no stale spike.
    rope_dist_prev_ = -1.0;
    rope_dist_dot_ = 0.0;
    last_update_time_ = data->time;
    return;
  }

  // Finite-difference rope-extension rate (taut branch only), EMA-smoothed.
  // Raw d(rope_dist)/dt over 2 ms is noisy due to contact/joint vibrations; the
  // EMA (α≈0.2, τ≈10 ms) keeps low-frequency fall/bounce dynamics while
  // filtering out high-frequency noise that would otherwise cause large damp spikes.
  if (rope_dist_prev_ >= 0.0 && last_update_time_ >= 0.0)
  {
    const double dt = data->time - last_update_time_;
    if (dt > 1e-9)
    {
      const double raw_dot = (rope_dist - rope_dist_prev_) / dt;
      const double alpha = std::min(1.0, dt / 0.01);  // τ = 10 ms
      rope_dist_dot_ = alpha * raw_dot + (1.0 - alpha) * rope_dist_dot_;
    }
  }
  rope_dist_prev_ = rope_dist;
  last_update_time_ = data->time;

  const double rope_dir[3] = { dx / rope_dist, dy / rope_dist, dz / rope_dist };

  // Spring always pulls toward anchor when taut; damping only resists extension.
  // Bidirectional damping (max(0, spring+damp)) silences the spring when contracting
  // fast, which creates an asymmetric energy cycle and growing oscillations.
  const double extension = rope_dist - rope_length_;
  const double damp = (rope_dist_dot_ > 0.0) ? kd_pos_ * rope_dist_dot_ : 0.0;
  const double tension = kp_pos_ * extension + damp;

  // Force toward anchor (opposite of rope_dir).
  const double Fx = -tension * rope_dir[0];
  const double Fy = -tension * rope_dir[1];
  const double Fz = -tension * rope_dir[2];

  data->xfrc_applied[body_id_ * 6 + 0] = Fx;
  data->xfrc_applied[body_id_ * 6 + 1] = Fy;
  data->xfrc_applied[body_id_ * 6 + 2] = Fz;

  // Torque correction for off-CoM attachment: τ = offset_world × F.
  const double ox = xmat[0] * body_offset_[0] + xmat[1] * body_offset_[1] + xmat[2] * body_offset_[2];
  const double oy = xmat[3] * body_offset_[0] + xmat[4] * body_offset_[1] + xmat[5] * body_offset_[2];
  const double oz = xmat[6] * body_offset_[0] + xmat[7] * body_offset_[1] + xmat[8] * body_offset_[2];

  data->xfrc_applied[body_id_ * 6 + 3] = oy * Fz - oz * Fy;
  data->xfrc_applied[body_id_ * 6 + 4] = oz * Fx - ox * Fz;
  data->xfrc_applied[body_id_ * 6 + 5] = ox * Fy - oy * Fx;
}

void VirtualGantryPlugin::reset()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  spawn_pos_captured_ = false;
  rope_dist_prev_ = -1.0;
  rope_dist_dot_ = 0.0;
  last_update_time_ = -1.0;
}

void VirtualGantryPlugin::update_visualization(const mjModel* /*model*/, const mjData* data, mjvScene* scene)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!enabled_ || !spawn_pos_captured_ || body_id_ < 0 || !data || !data->xpos || !data->xmat || !scene ||
      !scene->geoms ||
      scene->ngeom >= scene->maxgeom)
  {
    return;
  }

  const double* xpos = &data->xpos[body_id_ * 3];
  const double* xmat = &data->xmat[body_id_ * 9];

  mjtNum attach_pos[3];
  for (int i = 0; i < 3; ++i)
  {
    attach_pos[i] = xpos[i] + xmat[i * 3 + 0] * body_offset_[0] + xmat[i * 3 + 1] * body_offset_[1] +
                    xmat[i * 3 + 2] * body_offset_[2];
  }

  constexpr mjtNum kVisualizationLateralOffset = 0.18;
  const mjtNum anchor_pos[3] = { anchor_pos_[0], anchor_pos_[1] + kVisualizationLateralOffset, anchor_pos_[2] };
  attach_pos[1] += kVisualizationLateralOffset;
  const float rope_rgba[4] = { 0.1f, 0.95f, 0.2f, 1.0f };

  mjvGeom* geom = &scene->geoms[scene->ngeom++];
  mjv_initGeom(geom, mjGEOM_CAPSULE, nullptr, nullptr, nullptr, rope_rgba);
  geom->category = mjCAT_DECOR;
  mjv_connector(geom, mjGEOM_CAPSULE, 0.025, anchor_pos, attach_pos);

  if (scene->ngeom >= scene->maxgeom)
  {
    return;
  }

  const mjtNum anchor_size[3] = { 0.08, 0.0, 0.0 };
  const float anchor_rgba[4] = { 1.0f, 0.85f, 0.05f, 1.0f };
  mjvGeom* anchor_geom = &scene->geoms[scene->ngeom++];
  mjv_initGeom(anchor_geom, mjGEOM_SPHERE, anchor_size, anchor_pos, nullptr, anchor_rgba);
  anchor_geom->category = mjCAT_DECOR;
}

bool VirtualGantryPlugin::on_key(int key, int /*scancode*/, int action, int /*mods*/)
{
  // GLFW constants (stable values matching glfw3.h; avoids adding glfw as a plugin dependency).
  constexpr int kPress = 1;
  constexpr int kRepeat = 2;
  constexpr int kKeyG = 71;
  constexpr int kKeyLeftBracket = 91;
  constexpr int kKeyRightBracket = 93;

  if (action != kPress && action != kRepeat)
  {
    return false;
  }
  if (key == kKeyG && action == kPress)
  {
    toggle_counter_.fetch_add(1, std::memory_order_release);
    return true;
  }
  if (key == kKeyLeftBracket)
  {
    rope_length_ticks_.fetch_add(-1, std::memory_order_release);
    return true;
  }
  if (key == kKeyRightBracket)
  {
    rope_length_ticks_.fetch_add(1, std::memory_order_release);
    return true;
  }
  return false;
}

void VirtualGantryPlugin::cleanup()
{
  // Hold state_mutex_ so any in-flight service callback (which also takes the lock and
  // dereferences node_) finishes before we tear down the service and node references.
  std::lock_guard<std::mutex> lock(state_mutex_);
  enable_srv_.reset();
  node_.reset();
}

}  // namespace mujoco_ros2_control_plugins

PLUGINLIB_EXPORT_CLASS(mujoco_ros2_control_plugins::VirtualGantryPlugin,
                       mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
