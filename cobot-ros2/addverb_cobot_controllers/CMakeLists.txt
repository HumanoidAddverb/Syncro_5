cmake_minimum_required(VERSION 3.8)
project(addverb_cobot_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(utility REQUIRED)

set(COBOT_UI_BACKEND_DIR /opt/addverb/cobot_backend)



find_package(controller_manager REQUIRED)

find_package(backward_ros REQUIRED)

find_package(addverb_cobot_msgs REQUIRED)

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp_components REQUIRED)

find_package(tf2_ros REQUIRED)

find_package(tf2_geometry_msgs REQUIRED)

find_package(kdl_parser REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  control_msgs
  std_msgs
  geometry_msgs
  trajectory_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  controller_interface
  rclcpp_action
  addverb_cobot_msgs
  rclcpp_components
  realtime_tools
  tf2_ros
  tf2_geometry_msgs
  kdl_parser  
)


# Function to glob files based on a pattern
# TODO: Move these functions to a common cmake
function(glob_files GLOB_PATTERN OUTPUT_VAR)
    # Get the list of files with full paths based on the provided pattern
    if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.11.0")
      file(GLOB_RECURSE FILE_LIST CONFIGURE_DEPENDS ${GLOB_PATTERN})
    else()
      # For backwards compatibility
      file(GLOB FILE_LIST ${GLOB_PATTERN})
    endif()

    # Set the output variable to the list of files
    set(${OUTPUT_VAR} ${FILE_LIST} PARENT_SCOPE)
endfunction()

# Define a function to collect unique directory paths from a glob pattern
function(get_unique_dirs_from_glob GLOB_PATTERN OUTPUT_VAR)
    # Call the glob_files function to get the list of files
    glob_files(${GLOB_PATTERN} FILE_LIST)

    # Initialize a list to store unique directory paths
    set(UNIQUE_DIRS)

    # Loop over each file path and extract its directory path
    foreach(FILE_PATH ${FILE_LIST})
        get_filename_component(DIR_PATH ${FILE_PATH} DIRECTORY)
        list(APPEND UNIQUE_DIRS ${DIR_PATH})
    endforeach()

    # Remove duplicate directory paths
    list(REMOVE_DUPLICATES UNIQUE_DIRS)

    # Set the output variable to the list of unique directory paths
    set(${OUTPUT_VAR} ${UNIQUE_DIRS} PARENT_SCOPE)
endfunction()



# Look for header files and get their locations
get_unique_dirs_from_glob("${COBOT_UI_BACKEND_DIR}/**.h" COBOT_UI_BACKEND_INCLUDE_DIRS)


# Get all shared objects
glob_files("${COBOT_UI_BACKEND_DIR}/**.so" COBOT_UI_BACKEND_LIBRARIES)


# # Look for header files and get their locations
# get_unique_dirs_from_glob("${COBOT_UI_BACKEND_DIR}/src/*.h" COBOT_UI_BACKEND_INCLUDE_DIRS)

# # Get all shared objects
# glob_files("${COBOT_UI_BACKEND_DIR}/build/*.so" COBOT_UI_BACKEND_LIBRARIES)

add_library(controller_utils SHARED src/controller_utils.cpp)

target_include_directories(controller_utils 
PUBLIC 
$<BUILD_INTERFACE:include/
${COBOT_UI_BACKEND_INCLUDE_DIRS}>
$<INSTALL_INTERFACE:include/addverb_cobot_controllers>
)

ament_target_dependencies(
  controller_utils PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  utility
)

add_library(
  addverb_cobot_controllers
  SHARED
  src/ptp_joint_controller.cpp
  src/free_drive_controller.cpp
  src/ptp_tcp_controller.cpp
  src/recorder_controller.cpp
  src/joint_jogging_controller.cpp
  src/cartesian_jogging_controller.cpp
  src/joint_impedance_controller.cpp
  src/cartesian_impedance_controller.cpp
  src/gripper_controller.cpp
  src/gravity_comp_effort_controller.cpp
  src/effort_controller.cpp
  src/velocity_controller.cpp
)


target_include_directories(addverb_cobot_controllers PUBLIC
$<BUILD_INTERFACE:include/
${COBOT_UI_BACKEND_INCLUDE_DIRS}>
$<INSTALL_INTERFACE:include/addverb_cobot_controllers>
)

target_link_libraries(addverb_cobot_controllers PUBLIC 
${COBOT_UI_BACKEND_LIBRARIES}
controller_utils
)

ament_target_dependencies(
  addverb_cobot_controllers PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  utility
)

# Export controllers
pluginlib_export_plugin_description_file(controller_interface addverb_cobot_controllers_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(TARGETS addverb_cobot_controllers  controller_utils
  EXPORT export_addverb_cobot_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(gripper_action_bridge src/gripper_action_bridge.cpp)

ament_target_dependencies(
  gripper_action_bridge
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  addverb_cobot_msgs
)

target_include_directories(gripper_action_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS gripper_action_bridge
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## EXPORTS
ament_export_targets(export_addverb_cobot_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()