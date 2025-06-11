// Copyright 2020 PAL Robotics SL.
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

#include <gmock/gmock.h>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

const std::string valid_swerve_urdf =
  R"(
<?xml version="1.0" ?>
<robot name="swerve_bot">
    <link name="base_footprint"></link>
  <joint name="base_joint" type="fixed">
    <parent link="base_footprint" />
    <child link="base_link" />
    <origin xyz="0 0 0.5" rpy="0 0 0" />
  </joint>
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="2 2 0.5" />
      </geometry>
      <material name="base_link-material">
        <color rgba="0.5775804404214573 0.001214107934117647 0.3915724777393922 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <box size="2 2 0.5" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="1" />
      <inertia ixx="0.16666666666666666" ixy="0" ixz="0" iyy="0.16666666666666666" iyz="0"
        izz="0.16666666666666666" />
    </inertial>
  </link>
  <joint name="fl_steer_joint" type="revolute">
    <parent link="base_link" />
    <child link="fl_steer_link" />
    <origin xyz="0.9 1.15 0" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit effort="1000.0" lower="${-M_PI}" upper="${M_PI}" velocity="0.5" />
  </joint>
  <link name="fl_steer_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.1" length="0.5" />
      </geometry>
      <material name="fl_steer_link-material">
        <color rgba="0.13563332964548108 0.775822218312646 0.822785754392438 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.1" length="0.5" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="1" />
      <inertia ixx="0.3333333333333333" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.3333333333333333" />
    </inertial>
  </link>
  <joint name="fl_drive_joint" type="continuous">
    <parent link="fl_steer_link" />
    <child link="fl_drive_link" />
    <origin xyz="0 0 -0.475" rpy="1.5707963267948963 0 0" />
    <axis xyz="0 0 -1" />
  </joint>
  <link name="fl_drive_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.2" length="0.2" />
      </geometry>
      <material name="fl_drive_link-material">
        <color rgba="0.04666508633021928 0.07227185067438519 0.2541520943200296 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.2" length="0.2" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="1" />
      <inertia ixx="0.3333333333333333" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.3333333333333333" />
    </inertial>
  </link>
  <joint name="fr_steer_joint" type="revolute">
    <parent link="base_link" />
    <child link="fr_steer_link" />
    <origin xyz="0.9 -1.15 0" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit effort="1000.0" lower="${-M_PI}" upper="${M_PI}" velocity="0.5" />
  </joint>
  <link name="fr_steer_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.1" length="0.5" />
      </geometry>
      <material name="fr_steer_link-material">
        <color rgba="0.13563332964548108 0.775822218312646 0.822785754392438 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.1" length="0.5" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="1" />
      <inertia ixx="0.3333333333333333" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.3333333333333333" />
    </inertial>
  </link>
  <joint name="fr_drive_joint" type="continuous">
    <parent link="fr_steer_link" />
    <child link="fr_driver_link" />
    <origin xyz="0 0 -0.475" rpy="1.5707963267948963 0 0" />
    <axis xyz="0 0 -1" />
  </joint>
  <link name="fr_driver_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.2" length="0.2" />
      </geometry>
      <material name="fr_driver_link-material">
        <color rgba="0.04666508633021928 0.07227185067438519 0.2541520943200296 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.2" length="0.2" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="1" />
      <inertia ixx="0.3333333333333333" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.3333333333333333" />
    </inertial>
  </link>
  <joint name="rl_steer_joint" type="revolute">
    <parent link="base_link" />
    <child link="rl_steer_link" />
    <origin xyz="-0.9 1.15 0" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit effort="1000.0" lower="${-M_PI}" upper="${M_PI}" velocity="0.5" />
  </joint>
  <link name="rl_steer_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.1" length="0.5" />
      </geometry>
      <material name="rl_steer_link-material">
        <color rgba="0.13563332964548108 0.775822218312646 0.822785754392438 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.1" length="0.5" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="1" />
      <inertia ixx="0.3333333333333333" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.3333333333333333" />
    </inertial>
  </link>
  <joint name="rl_drive_joint" type="continuous">
    <parent link="rl_steer_link" />
    <child link="rl_driver_link" />
    <origin xyz="0 0 -0.475" rpy="1.5707963267948963 0 0" />
    <axis xyz="0 0 -1" />
  </joint>
  <link name="rl_driver_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.2" length="0.2" />
      </geometry>
      <material name="rl_driver_link-material">
        <color rgba="0.04666508633021928 0.07227185067438519 0.2541520943200296 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.2" length="0.2" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="1" />
      <inertia ixx="0.3333333333333333" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.3333333333333333" />
    </inertial>
  </link>
  <joint name="rr_steer_joint" type="revolute">
    <parent link="base_link" />
    <child link="rr_steer_link" />
    <origin xyz="-0.9 -1.15 0" rpy="0 0 0" />
    <axis xyz="0 0 1" />
    <limit effort="1000.0" lower="${-M_PI}" upper="${M_PI}" velocity="0.5" />
  </joint>
  <link name="rr_steer_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.1" length="0.5" />
      </geometry>
      <material name="rr_steer_link-material">
        <color rgba="0.13563332964548108 0.775822218312646 0.822785754392438 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.1" length="0.5" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="1" />
      <inertia ixx="0.3333333333333333" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.3333333333333333" />
    </inertial>
  </link>
  <joint name="rr_drive_joint" type="continuous">
    <parent link="rr_steer_link" />
    <child link="rr_driver_link" />
    <origin xyz="0 0 -0.475" rpy="1.5707963267948963 0 0" />
    <axis xyz="0 0 -1" />
  </joint>
  <link name="rr_driver_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.2" length="0.2" />
      </geometry>
      <material name="rr_driver_link-material">
        <color rgba="0.04666508633021928 0.07227185067438519 0.2541520943200296 1" />
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <geometry>
        <cylinder radius="0.2" length="0.2" />
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0" />
      <mass value="1" />
      <inertia ixx="0.3333333333333333" ixy="0" ixz="0" iyy="0.5" iyz="0" izz="0.3333333333333333" />
    </inertial>
  </link>
  <ros2_control name="TestFlDriveActuatorHardware" type="actuator">
    <hardware>
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name="fl_drive_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  <ros2_control name="TestFrDriveActuatorHardware" type="actuator">
    <hardware>
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name="fr_drive_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  <ros2_control name="TestRlDriveActuatorHardware" type="actuator">
    <hardware>
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name="rl_drive_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  <ros2_control name="TestRrDriveActuatorHardware" type="actuator">
    <hardware>
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name="rr_drive_joint">
      <command_interface name="velocity"/>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
  <ros2_control name="TestFlSteerActuatorHardware" type="actuator">
    <hardware>
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name="fl_steer_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
  <ros2_control name="TestFrSteerActuatorHardware" type="actuator">
    <hardware>
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name="fr_steer_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
  <ros2_control name="TestRlSteerActuatorHardware" type="actuator">
    <hardware>
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name="rl_steer_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
  <ros2_control name="TestRrSteerActuatorHardware" type="actuator">
    <hardware>
      <plugin>test_actuator</plugin>
    </hardware>
    <joint name="rr_steer_joint">
      <command_interface name="position"/>
      <state_interface name="position"/>
    </joint>
  </ros2_control>
</robot>
)";

TEST(TestLoadSwerveController, load_controller)
{
  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  controller_manager::ControllerManager cm(
    std::make_unique<hardware_interface::ResourceManager>(valid_swerve_urdf),
    executor, "test_controller_manager");

  ASSERT_NE(
    cm.load_controller("test_swerve_controller", "swerve_controller/SwerverController"),
    nullptr);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  rclcpp::shutdown();
  return result;
}
