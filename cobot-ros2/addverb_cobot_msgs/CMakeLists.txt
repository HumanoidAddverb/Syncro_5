cmake_minimum_required(VERSION 3.8)
project(addverb_cobot_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)



find_package(rosidl_default_generators REQUIRED)


# In addverb_cobot_msgs/CMakeLists.txt

rosidl_generate_interfaces(${PROJECT_NAME}
  # Base message with no internal dependencies
  "msg/CartesianPoint.msg"

  # # Depends on CartesianPoint
  "msg/CartesianTrajectoryPoint.msg"

  # # Depends on CartesianTrajectoryPoint
  "msg/CartesianTrajectory.msg"

  # # Depends on CartesianTrajectory
  "action/FollowCartesianTrajectory.action"
  # Other actions/services that are independent or depend on the above
  "action/Replay.action"
  "srv/Record.srv"
  "srv/FlexFactor.srv"
  "srv/Gripper.srv"
  "msg/JointJoggingVelocity.msg"


  # External package dependencies remain the same
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
