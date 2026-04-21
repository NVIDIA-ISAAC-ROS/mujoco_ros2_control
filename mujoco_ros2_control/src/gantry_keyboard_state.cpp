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

#include "mujoco_ros2_control_plugins/gantry_keyboard_state.hpp"

namespace mujoco_ros2_control_plugins
{

// Defined in libmujoco_ros2_control.so (not in the plugins DSO) so there is
// exactly one instance. visibility("default") ensures the symbol is exported
// even when -fvisibility=hidden is active, so the plugins DSO can resolve it
// from the global symbol table at dlopen time.
__attribute__((visibility("default")))
GantryKeyboardState & GantryKeyboardState::get()
{
  static GantryKeyboardState instance;
  return instance;
}

}  // namespace mujoco_ros2_control_plugins
