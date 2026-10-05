/**
 * Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

namespace mujoco_ros2_control
{

/** The four coefficients used by MuJoCo's affine actuator gain/bias model. */
struct ActuatorAffineParameters
{
  double gain{ 0.0 };
  double bias_constant{ 0.0 };
  double bias_position{ 0.0 };
  double bias_velocity{ 0.0 };

  bool operator==(const ActuatorAffineParameters& other) const
  {
    return gain == other.gain && bias_constant == other.bias_constant && bias_position == other.bias_position &&
           bias_velocity == other.bias_velocity;
  }

  bool operator!=(const ActuatorAffineParameters& other) const
  {
    return !(*this == other);
  }
};

/** A complete affine-parameter update for one MuJoCo actuator. */
struct ActuatorParameterUpdate
{
  int actuator_id;
  ActuatorAffineParameters parameters;
};

}  // namespace mujoco_ros2_control
