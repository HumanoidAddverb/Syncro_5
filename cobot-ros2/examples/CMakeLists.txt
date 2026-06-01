cmake_minimum_required(VERSION 3.8)
project(examples)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(addverb_cobot_msgs REQUIRED)
find_package(control_msgs REQUIRED)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

add_executable(demo_velocity src/demo_velocity.cpp)
ament_target_dependencies(demo_velocity rclcpp std_msgs)

add_executable(demo_effort src/demo_effort.cpp)
ament_target_dependencies(demo_effort rclcpp geometry_msgs)

add_executable(demo_gravity_comp_effort src/demo_gravity_comp_effort.cpp)
ament_target_dependencies(demo_gravity_comp_effort rclcpp geometry_msgs)

add_executable(demo_joint_jogging src/demo_joint_jogging.cpp)
ament_target_dependencies(demo_joint_jogging rclcpp addverb_cobot_msgs)

add_executable(demo_cartesian_jogging src/demo_cartesian_jogging.cpp)
ament_target_dependencies(demo_cartesian_jogging rclcpp addverb_cobot_msgs geometry_msgs)

add_executable(demo_gripper src/demo_gripper.cpp)
ament_target_dependencies(demo_gripper rclcpp addverb_cobot_msgs geometry_msgs)

add_executable(demo_joint_impedance src/demo_joint_impedance.cpp)
ament_target_dependencies(demo_joint_impedance rclcpp rclcpp_action rclcpp_components addverb_cobot_msgs control_msgs)

add_executable(demo_cartesian_impedance src/demo_cartesian_impedance.cpp)
ament_target_dependencies(demo_cartesian_impedance rclcpp rclcpp_action rclcpp_components addverb_cobot_msgs control_msgs)

add_executable(demo_ptp_tcp src/demo_ptp_tcp.cpp)
ament_target_dependencies(demo_ptp_tcp rclcpp rclcpp_action rclcpp_components addverb_cobot_msgs)

add_executable(demo_recorder src/demo_recorder.cpp)
ament_target_dependencies(demo_recorder rclcpp rclcpp_action rclcpp_components addverb_cobot_msgs)

add_executable(dual_arm_joint_jogging_test src/dual_arm/test_joint_jogging.cpp)
ament_target_dependencies(dual_arm_joint_jogging_test rclcpp addverb_cobot_msgs)

add_executable(dual_arm_external_velocity_test src/dual_arm/test_external_velocity.cpp)
ament_target_dependencies(dual_arm_external_velocity_test rclcpp addverb_cobot_msgs)

add_executable(dual_arm_cartesian_jogging_test src/dual_arm/test_cartesian_jogging.cpp)
ament_target_dependencies(dual_arm_cartesian_jogging_test rclcpp addverb_cobot_msgs geometry_msgs)

add_executable(dual_arm_test_ptp src/dual_arm/test_ptp.cpp)
ament_target_dependencies(dual_arm_test_ptp rclcpp addverb_cobot_msgs geometry_msgs rclcpp_action)

add_executable(dual_arm_gravity_comp_effort src/dual_arm/test_gravity_comp_effort.cpp)
ament_target_dependencies(dual_arm_gravity_comp_effort rclcpp addverb_cobot_msgs)

add_executable(dual_arm_joint_impedance_test src/dual_arm/test_joint_impedance.cpp)
ament_target_dependencies(dual_arm_joint_impedance_test rclcpp rclcpp_action rclcpp_components addverb_cobot_msgs control_msgs)

add_executable(dual_arm_cartesian_impedance_test src/dual_arm/test_cartesian_impedance.cpp)
ament_target_dependencies(dual_arm_cartesian_impedance_test rclcpp rclcpp_action rclcpp_components addverb_cobot_msgs control_msgs)

add_executable(dual_arm_recorder_test src/dual_arm/test_recorder.cpp)
ament_target_dependencies(dual_arm_recorder_test rclcpp rclcpp_action rclcpp_components addverb_cobot_msgs)

add_executable(save_waypoints src/dual_arm/save_waypoints.cpp)
ament_target_dependencies(save_waypoints rclcpp rclcpp_action rclcpp_components addverb_cobot_msgs control_msgs)

add_executable(demo_pick_and_place src/dual_arm/demo_pick_and_place.cpp)
ament_target_dependencies(demo_pick_and_place rclcpp rclcpp_action rclcpp_components addverb_cobot_msgs control_msgs geometry_msgs)

############## PTP 
add_library(ptp_demo_client SHARED src/demo_ptp_joint.cpp)
ament_target_dependencies(ptp_demo_client
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "trajectory_msgs"
  "control_msgs")

rclcpp_components_register_node(ptp_demo_client PLUGIN "ptp::DemoClient" EXECUTABLE demo_ptp_joint)

add_library(dual_arm_joint_ptp_test SHARED src/dual_arm/test_joint_ptp.cpp)

ament_target_dependencies(dual_arm_joint_ptp_test
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "trajectory_msgs"
  "control_msgs")

rclcpp_components_register_node(dual_arm_joint_ptp_test PLUGIN "ptp::DualArmPTPClient" EXECUTABLE dual_arm_joint_ptp)

install(TARGETS
  demo_velocity
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  demo_effort
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  demo_gravity_comp_effort
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  demo_joint_jogging
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  demo_cartesian_jogging
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  demo_joint_impedance
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  demo_gripper
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  demo_recorder
  DESTINATION lib/${PROJECT_NAME})

  install(TARGETS
  demo_ptp_tcp
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  demo_cartesian_impedance
  DESTINATION lib/${PROJECT_NAME})
  
install(TARGETS
  dual_arm_joint_jogging_test
  dual_arm_external_velocity_test
  dual_arm_cartesian_jogging_test
  dual_arm_test_ptp
  dual_arm_gravity_comp_effort
  dual_arm_joint_impedance_test
  dual_arm_cartesian_impedance_test
  dual_arm_recorder_test
  save_waypoints
  demo_pick_and_place
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
ptp_demo_client
dual_arm_joint_ptp_test
ARCHIVE DESTINATION lib
LIBRARY DESTINATION lib
RUNTIME DESTINATION bin)


ament_package()
