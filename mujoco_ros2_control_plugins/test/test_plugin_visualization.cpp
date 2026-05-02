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

#include <gtest/gtest.h>

#include <array>

#include <mujoco/mujoco.h>

#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

#define private public
#include "mujoco_ros2_control_plugins/virtual_gantry_plugin.hpp"
#undef private

namespace
{

class MinimalPlugin : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr /*node*/, const mjModel* /*model*/, mjData* /*data*/) override
  {
    return true;
  }

  void update(const mjModel* /*model*/, mjData* /*data*/) override
  {
  }

  void cleanup() override
  {
  }
};

TEST(PluginVisualizationTest, BaseHookDoesNotAddVisualizationGeoms)
{
  MinimalPlugin plugin;
  mjvScene scene;
  mjv_defaultScene(&scene);
  scene.ngeom = 7;

  plugin.update_visualization(nullptr, nullptr, &scene);

  EXPECT_EQ(scene.ngeom, 7);
}

struct GantryVisualizationFixture
{
  mujoco_ros2_control_plugins::VirtualGantryPlugin plugin;
  mjModel model{};
  mjData data{};
  std::array<mjtNum, 3> xpos{ { 1.0, 2.0, 0.8 } };
  std::array<mjtNum, 9> xmat{ { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 } };
  std::array<mjvGeom, 3> geoms{};
  std::array<int, 3> geomorder{};
  mjvScene scene{};

  GantryVisualizationFixture()
  {
    data.xpos = xpos.data();
    data.xmat = xmat.data();

    mjv_defaultScene(&scene);
    scene.maxgeom = static_cast<int>(geoms.size());
    scene.geoms = geoms.data();
    scene.geomorder = geomorder.data();

    plugin.body_id_ = 0;
    plugin.body_offset_ = { { -0.16, 0.0, 0.3 } };
    plugin.anchor_pos_ = { { 0.84, 2.0, 1.5 } };
    plugin.enabled_ = true;
    plugin.spawn_pos_captured_ = true;
  }
};

TEST(PluginVisualizationTest, GantryDoesNotDrawWhenDisabled)
{
  GantryVisualizationFixture fixture;
  fixture.plugin.enabled_ = false;

  fixture.plugin.update_visualization(&fixture.model, &fixture.data, &fixture.scene);

  EXPECT_EQ(fixture.scene.ngeom, 0);
}

TEST(PluginVisualizationTest, GantryDoesNotDrawBeforeAnchorCapture)
{
  GantryVisualizationFixture fixture;
  fixture.plugin.spawn_pos_captured_ = false;

  fixture.plugin.update_visualization(&fixture.model, &fixture.data, &fixture.scene);

  EXPECT_EQ(fixture.scene.ngeom, 0);
}

TEST(PluginVisualizationTest, GantryDrawsRopeAndAnchorWhenEnabledAndCaptured)
{
  GantryVisualizationFixture fixture;

  fixture.plugin.update_visualization(&fixture.model, &fixture.data, &fixture.scene);

  ASSERT_EQ(fixture.scene.ngeom, 2);
  EXPECT_EQ(fixture.scene.geoms[0].type, mjGEOM_CAPSULE);
  EXPECT_EQ(fixture.scene.geoms[1].type, mjGEOM_SPHERE);
  EXPECT_NEAR(fixture.scene.geoms[0].pos[0], 0.84, 1e-6);
  EXPECT_NEAR(fixture.scene.geoms[0].pos[1], 2.0, 1e-6);
  EXPECT_NEAR(fixture.scene.geoms[1].pos[0], 0.84, 1e-6);
  EXPECT_NEAR(fixture.scene.geoms[1].pos[1], 2.0, 1e-6);
  EXPECT_NEAR(fixture.scene.geoms[1].size[0], 0.08, 1e-6);
}

TEST(PluginVisualizationTest, GantryStillDrawsRopeWhenOnlyOneVisualizationGeomIsAvailable)
{
  GantryVisualizationFixture fixture;
  fixture.scene.maxgeom = 1;

  fixture.plugin.update_visualization(&fixture.model, &fixture.data, &fixture.scene);

  ASSERT_EQ(fixture.scene.ngeom, 1);
  EXPECT_EQ(fixture.scene.geoms[0].type, mjGEOM_CAPSULE);
}

}  // namespace
