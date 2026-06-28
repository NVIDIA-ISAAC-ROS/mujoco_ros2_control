// Copyright (C) 2026 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//         http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Authors: Julia Jia

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>

#include <hardware_interface/version.h>
#include <mujoco/mujoco.h>
#include <hardware_interface/hardware_info.hpp>
#include <mujoco_ros2_control/mujoco_system_interface.hpp>
#include <mujoco_ros2_control/sim_display_text.hpp>
#include <rclcpp/rclcpp.hpp>

#define ROS_DISTRO_HUMBLE (HARDWARE_INTERFACE_VERSION_MAJOR < 3)

class HeadlessInitTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok())
    {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite()
  {
    if (rclcpp::ok())
    {
      rclcpp::shutdown();
    }
  }

  void SetUp() override
  {
    // Create a simple MuJoCo model
    create_test_model();

    // Initialize hardware interface
    hardware_info_ = create_hardware_info();
    interface_ = std::make_shared<mujoco_ros2_control::MujocoSystemInterface>();
  }

  void TearDown() override
  {
    // Deactivate interface before destroying to stop threads cleanly
    if (interface_)
    {
      rclcpp_lifecycle::State inactive_state(0, "inactive");
      interface_->on_deactivate(inactive_state);
      interface_.reset();
    }

    // Clean up test file
    if (std::filesystem::exists(test_model_path_))
    {
      std::filesystem::remove(test_model_path_);
    }
  }

  void create_test_model()
  {
    // Create a simple MuJoCo XML with two bodies and a floor
    test_model_path_ = "/tmp/test_headless_init_model.xml";
    std::ofstream file(test_model_path_);
    file << R"(<?xml version="1.0"?>
<mujoco model="test_headless_init">
  <option timestep="0.002"/>

  <size nconmax="100"/>

  <worldbody>
    <!-- Floor as infinite plane directly in worldbody (static by default) -->
    <geom name="floor_geom" type="plane" size="0 0 1"
          contype="1" conaffinity="1"/>

    <body name="box1" pos="0 0 0.1">
      <freejoint name="box1_joint"/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="box1_geom" type="box" size="0.05 0.05 0.05"
            contype="1" conaffinity="1" friction="0.6"/>
    </body>

    <body name="box2" pos="0.2 0 0.1">
      <freejoint/>
      <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>
      <geom name="box2_geom" type="box" size="0.05 0.05 0.05"
            contype="1" conaffinity="1" friction="0.6"/>
      <site name="site1"/>
    </body>
  </worldbody>

  <actuator>
    <velocity name="actuator1" site="site1"/>
  </actuator>
</mujoco>
)";
    file.close();
  }

  hardware_interface::HardwareInfo create_hardware_info()
  {
    hardware_interface::HardwareInfo info;
    info.name = "test_mujoco";
    info.type = "system";
    info.hardware_parameters["mujoco_model"] = test_model_path_;
    info.hardware_parameters["meshdir"] = "";
    info.hardware_parameters["headless"] = "true";           // Enable headless mode for CI compatibility
    info.hardware_parameters["disable_rendering"] = "true";  // Disable cameras/lidar to avoid OpenGL issues in tests

    return info;
  }

  hardware_interface::HardwareInfo create_impedance_hardware_info()
  {
    std::ofstream file(test_model_path_);
    file << R"(<?xml version="1.0"?>
<mujoco model="test_impedance_control">
  <option timestep="0.002" gravity="0 0 0"/>
  <worldbody>
    <body name="link" pos="0 0 0">
      <joint name="hinge" type="hinge" axis="0 1 0"/>
      <geom type="capsule" size="0.02" fromto="0 0 0 0.3 0 0" mass="1"/>
    </body>
  </worldbody>
  <actuator>
    <motor name="hinge" joint="hinge"/>
  </actuator>
</mujoco>
)";
    file.close();

    hardware_interface::HardwareInfo info;
    info.name = "test_mujoco_impedance";
    info.type = "system";
#if !ROS_DISTRO_HUMBLE
    info.rw_rate = 100;
#endif
    info.hardware_parameters["mujoco_model"] = test_model_path_;
    info.hardware_parameters["headless"] = "true";

    hardware_interface::ComponentInfo joint;
    joint.name = "hinge";
    joint.type = "joint";
    const auto make_interface = [](const std::string& name) {
      hardware_interface::InterfaceInfo interface;
      interface.name = name;
      interface.size = 1;
#if !ROS_DISTRO_HUMBLE
      interface.enable_limits = false;
#endif
      return interface;
    };
    joint.state_interfaces = {
      make_interface(hardware_interface::HW_IF_POSITION),
      make_interface(hardware_interface::HW_IF_VELOCITY),
      make_interface(hardware_interface::HW_IF_EFFORT),
    };
    joint.command_interfaces = {
      make_interface(hardware_interface::HW_IF_POSITION), make_interface(hardware_interface::HW_IF_VELOCITY),
      make_interface(hardware_interface::HW_IF_EFFORT),   make_interface(mujoco_ros2_control::HW_IF_KP),
      make_interface(mujoco_ros2_control::HW_IF_KD),
    };
    info.joints.push_back(joint);
    return info;
  }

  std::string test_model_path_;
  hardware_interface::HardwareInfo hardware_info_;
  std::shared_ptr<mujoco_ros2_control::MujocoSystemInterface> interface_;
};

TEST_F(HeadlessInitTest, HeadlessInitialization)
{
  // Test that MujocoSystemInterface can be initialized in headless mode
#if ROS_DISTRO_HUMBLE
  auto result = interface_->on_init(hardware_info_);
#else
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = hardware_info_;
  auto result = interface_->on_init(params);
#endif
  ASSERT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);

  // Check that the data and model are available, meaning initializing was successful.
  auto start = std::chrono::steady_clock::now();
  auto timeout = std::chrono::seconds(1);
  mjModel* test_model = nullptr;
  mjData* test_data = nullptr;
  while (std::chrono::steady_clock::now() - start < timeout)
  {
    interface_->get_model(test_model);
    interface_->get_data(test_data);
    if (test_model != nullptr && test_data != nullptr)
    {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_NE(test_model, nullptr) << "Model failed to initialize within timeout";
  ASSERT_NE(test_data, nullptr) << "Model failed to initialize within timeout";

  // Test that we can export state interfaces without crashing
  auto state_interfaces = interface_->export_state_interfaces();
  EXPECT_GE(state_interfaces.size(), 0);
}

// Verify that on_activate() returns SUCCESS and does not start an OpenGL rendering thread when
// the model has no <camera> elements. Without the fix, glfwInit()/gladLoadGL would be called
// unconditionally and could crash on headless hosts that have GLFW but no usable GL driver.
TEST_F(HeadlessInitTest, HeadlessActivateWithoutCameras)
{
  hardware_interface::HardwareInfo info;
  info.name = "test_mujoco_no_cameras";
  info.type = "system";
  info.hardware_parameters["mujoco_model"] = test_model_path_;
  info.hardware_parameters["meshdir"] = "";
  info.hardware_parameters["headless"] = "true";
  // Intentionally omit "disable_rendering" — the fix must not rely on that workaround.

#if ROS_DISTRO_HUMBLE
  auto init_result = interface_->on_init(info);
#else
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  auto init_result = interface_->on_init(params);
#endif
  ASSERT_EQ(init_result, hardware_interface::CallbackReturn::SUCCESS);

  // Wait for model and data to be available.
  auto start = std::chrono::steady_clock::now();
  auto timeout = std::chrono::seconds(1);
  mjModel* test_model = nullptr;
  mjData* test_data = nullptr;
  while (std::chrono::steady_clock::now() - start < timeout)
  {
    interface_->get_model(test_model);
    interface_->get_data(test_data);
    if (test_model != nullptr && test_data != nullptr)
    {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_NE(test_model, nullptr) << "Model failed to initialize within timeout";
  ASSERT_NE(test_data, nullptr) << "Data failed to initialize within timeout";

  // ncam == 0 for the test model; on_activate must not launch a GLFW/OpenGL thread.
  rclcpp_lifecycle::State active_state(0, "active");
  auto activate_result = interface_->on_activate(active_state);
  EXPECT_EQ(activate_result, hardware_interface::CallbackReturn::SUCCESS);
}

TEST_F(HeadlessInitTest, SpeedFactorParamInitialization)
{
  hardware_info_.hardware_parameters["sim_speed_factor"] = "0.5";

#if ROS_DISTRO_HUMBLE
  auto result = interface_->on_init(hardware_info_);
#else
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = hardware_info_;
  auto result = interface_->on_init(params);
#endif
  ASSERT_EQ(result, hardware_interface::CallbackReturn::SUCCESS);

  // Wait for the model to load so the physics thread starts.
  auto start = std::chrono::steady_clock::now();
  mjModel* test_model = nullptr;
  mjData* test_data = nullptr;
  while (std::chrono::steady_clock::now() - start < std::chrono::seconds(2))
  {
    interface_->get_model(test_model);
    interface_->get_data(test_data);
    if (test_model != nullptr && test_data != nullptr)
    {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  ASSERT_NE(test_model, nullptr) << "Model failed to initialize within timeout";
  ASSERT_NE(test_data, nullptr) << "Data failed to initialize within timeout";

  // Let the physics thread run a few steps to confirm sim_speed_factor does not
  // cause a crash and the sim advances time.
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // Re-snapshot data after physics has had time to run (get_data copies the current state).
  interface_->get_data(test_data);

  // sim_time should have advanced from zero
  EXPECT_GT(test_data->time, 0.0) << "Simulation time did not advance with sim_speed_factor=0.5";
}

TEST_F(HeadlessInitTest, ImpedanceInterfacesDriveMotorControl)
{
  const auto info = create_impedance_hardware_info();
#if ROS_DISTRO_HUMBLE
  const auto init_result = interface_->on_init(info);
#else
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  const auto init_result = interface_->on_init(params);
#endif
  ASSERT_EQ(init_result, hardware_interface::CallbackReturn::SUCCESS);

  auto command_interfaces = interface_->export_command_interfaces();
  auto state_interfaces = interface_->export_state_interfaces();
  const auto find_command = [&command_interfaces](const std::string& name) -> hardware_interface::CommandInterface& {
    auto it = std::find_if(command_interfaces.begin(), command_interfaces.end(),
                           [&name](const auto& interface) { return interface.get_name() == name; });
    if (it == command_interfaces.end())
    {
      throw std::runtime_error("Missing command interface: " + name);
    }
    return *it;
  };
  const auto find_state = [&state_interfaces](const std::string& name) -> hardware_interface::StateInterface& {
    auto it = std::find_if(state_interfaces.begin(), state_interfaces.end(),
                           [&name](const auto& interface) { return interface.get_name() == name; });
    if (it == state_interfaces.end())
    {
      throw std::runtime_error("Missing state interface: " + name);
    }
    return *it;
  };

  ASSERT_EQ(interface_->read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
            hardware_interface::return_type::OK);
  double position_state;
  double velocity_state;
#if ROS_DISTRO_HUMBLE
  position_state = find_state("hinge/position").get_value();
  velocity_state = find_state("hinge/velocity").get_value();
#else
  const auto position_state_optional = find_state("hinge/position").get_optional<double>();
  const auto velocity_state_optional = find_state("hinge/velocity").get_optional<double>();
  ASSERT_TRUE(position_state_optional.has_value());
  ASSERT_TRUE(velocity_state_optional.has_value());
  position_state = *position_state_optional;
  velocity_state = *velocity_state_optional;
#endif

#if ROS_DISTRO_HUMBLE
  find_command("hinge/position").set_value(0.5);
  find_command("hinge/velocity").set_value(0.25);
  find_command("hinge/effort").set_value(0.1);
  find_command("hinge/kp").set_value(20.0);
  find_command("hinge/kd").set_value(4.0);
#else
  ASSERT_TRUE(find_command("hinge/position").set_value(0.5));
  ASSERT_TRUE(find_command("hinge/velocity").set_value(0.25));
  ASSERT_TRUE(find_command("hinge/effort").set_value(0.1));
  ASSERT_TRUE(find_command("hinge/kp").set_value(20.0));
  ASSERT_TRUE(find_command("hinge/kd").set_value(4.0));
#endif

  ASSERT_EQ(interface_->perform_command_mode_switch(
                { "hinge/position", "hinge/velocity", "hinge/effort", "hinge/kp", "hinge/kd" }, {}),
            hardware_interface::return_type::OK);
  ASSERT_EQ(interface_->write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
            hardware_interface::return_type::OK);

  const double expected_effort = 0.1 + 20.0 * (0.5 - position_state) + 4.0 * (0.25 - velocity_state);
  double observed_effort = 0.0;
  for (size_t attempt = 0; attempt < 1000; ++attempt)
  {
    mjData* data = nullptr;
    interface_->get_data(data);
    ASSERT_NE(data, nullptr);
    observed_effort = data->ctrl[0];
    mj_deleteData(data);
    if (std::abs(observed_effort - expected_effort) <= 1e-9)
    {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  EXPECT_NEAR(observed_effort, expected_effort, 1e-9);
}

TEST_F(HeadlessInitTest, LockstepWriteAdvancesConfiguredStepCount)
{
  auto info = create_hardware_info();
  info.hardware_parameters["lockstep"] = "true";
  info.hardware_parameters["lockstep_steps_per_update"] = "2";
#if ROS_DISTRO_HUMBLE
  const auto init_result = interface_->on_init(info);
#else
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  const auto init_result = interface_->on_init(params);
#endif
  ASSERT_EQ(init_result, hardware_interface::CallbackReturn::SUCCESS);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  mjData* before = nullptr;
  mjModel* model = nullptr;
  interface_->get_data(before);
  interface_->get_model(model);
  ASSERT_NE(before, nullptr);
  ASSERT_NE(model, nullptr);
  const double start_time = before->time;

  ASSERT_EQ(interface_->write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
            hardware_interface::return_type::OK);

  mjData* after = nullptr;
  interface_->get_data(after);
  ASSERT_NE(after, nullptr);
  EXPECT_NEAR(after->time, start_time + 2 * model->opt.timestep, 1e-9);
  mj_deleteData(before);
  mj_deleteData(after);
  mj_deleteModel(model);
}

TEST_F(HeadlessInitTest, FloatingBasePublishesConfiguredTransform)
{
  auto info = create_hardware_info();
  info.hardware_parameters["odom_free_joint_name"] = "box1_joint";
  info.hardware_parameters["odom_frame"] = "world";
  info.hardware_parameters["publish_floating_base_tf"] = "true";
#if ROS_DISTRO_HUMBLE
  const auto init_result = interface_->on_init(info);
#else
  hardware_interface::HardwareComponentInterfaceParams params;
  params.hardware_info = info;
  const auto init_result = interface_->on_init(params);
#endif
  ASSERT_EQ(init_result, hardware_interface::CallbackReturn::SUCCESS);

  auto listener = rclcpp::Node::make_shared("floating_base_tf_test_listener");
  std::atomic<bool> received{ false };
  auto subscription = listener->create_subscription<tf2_msgs::msg::TFMessage>(
      "/tf", 10, [&received](const tf2_msgs::msg::TFMessage& message) {
        for (const auto& transform : message.transforms)
        {
          if (transform.header.frame_id == "world" && transform.child_frame_id == "box1")
          {
            received.store(true);
          }
        }
      });

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  while (!received.load() && std::chrono::steady_clock::now() < deadline)
  {
    ASSERT_EQ(interface_->read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)),
              hardware_interface::return_type::OK);
    rclcpp::spin_some(listener);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  EXPECT_TRUE(received.load());
  (void)subscription;
}

TEST(SimDisplayTextTest, ComposesAllRowsWhenRunning)
{
  const auto [title, content] = mujoco_ros2_control::compose_sim_display_text(
      /*running=*/true, /*step_count=*/1234, /*sim_time=*/2.5,
      /*desired_pct=*/50.0, /*actual_pct=*/48.3, /*ncon=*/7);

  EXPECT_EQ(title, "Status\nSteps\nSim Time\nDesired Speed\nActual Speed\nContacts");
  EXPECT_EQ(content, "Running\n1234\n2.500 s\n50.0%\n48.3%\n7");

  // Title and content must line up row-for-row, otherwise the overlay is misaligned.
  const auto count_rows = [](const std::string& s) { return std::count(s.begin(), s.end(), '\n'); };
  EXPECT_EQ(count_rows(title), count_rows(content));
}

TEST(SimDisplayTextTest, ReportsPausedStatus)
{
  const auto [title, content] = mujoco_ros2_control::compose_sim_display_text(
      /*running=*/false, /*step_count=*/0, /*sim_time=*/0.0,
      /*desired_pct=*/100.0, /*actual_pct=*/0.0, /*ncon=*/0);

  EXPECT_EQ(content, "Paused\n0\n0.000 s\n100.0%\n0.0%\n0");
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  return result;
}
