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

#ifndef MUJOCO_ROS2_CONTROL_PLUGINS__GLFW_PLATFORM_HPP_
#define MUJOCO_ROS2_CONTROL_PLUGINS__GLFW_PLATFORM_HPP_

#include <cstdlib>

#include <GLFW/glfw3.h>

namespace mujoco_ros2_control_plugins
{

/**
 * GLFW >= 3.4 built with both Wayland and X11 probes Wayland first unless XDG_SESSION_TYPE says otherwise. Without
 * a Wayland compositor (e.g. inside Docker) libwayland prints "XDG_RUNTIME_DIR is invalid or not set" before GLFW
 * falls back to X11. Skip the probe when no Wayland display is advertised. Must run before glfwInit().
 *
 * Static so that each shared library calls the GLFW it links against; a statically linked GLFW has per-library state.
 */
static inline void prefer_x11_without_wayland_display()
{
#if GLFW_VERSION_MAJOR > 3 || (GLFW_VERSION_MAJOR == 3 && GLFW_VERSION_MINOR >= 4)
  if (std::getenv("WAYLAND_DISPLAY") == nullptr && glfwPlatformSupported(GLFW_PLATFORM_X11))
  {
    glfwInitHint(GLFW_PLATFORM, GLFW_PLATFORM_X11);
  }
#endif
}

}  // namespace mujoco_ros2_control_plugins

#endif  // MUJOCO_ROS2_CONTROL_PLUGINS__GLFW_PLATFORM_HPP_
