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

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__GANTRY_KEYBOARD_STATE_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__GANTRY_KEYBOARD_STATE_HPP_

#include <atomic>

namespace mujoco_ros2_control_plugins
{

// Singleton that bridges the GLFW UI thread (key/scroll events) and the
// VirtualGantryPlugin physics thread.  All fields are atomic — no mutex needed.
struct GantryKeyboardState
{
  // Incremented on each 'G' key press.  Plugin compares against its last-seen
  // value to detect a toggle edge without consuming the count.
  std::atomic<int> toggle_counter{0};

  // Accumulated Shift+scroll ticks.  Plugin exchanges to 0 after reading.
  // Positive = scroll up (lengthen rope), negative = scroll down (shorten).
  std::atomic<int> rope_scroll_ticks{0};

  static GantryKeyboardState & get()
  {
    static GantryKeyboardState instance;
    return instance;
  }

private:
  GantryKeyboardState() = default;
};

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__GANTRY_KEYBOARD_STATE_HPP_
