cmake_minimum_required(VERSION 3.8)
project(utility)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(addverb_cobot_msgs REQUIRED)

include_directories(include)

set(COBOT_UI_BACKEND_DIR /opt/addverb/cobot_backend)


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

add_library(utility_lib SHARED src/cobot_comm_wrapper.cpp
src/data_communicator.cpp
src/data_converter.cpp
src/data_processor.cpp
src/data_validator.cpp
src/hardware_interface_defs.cpp
src/finite_state_machine.cpp)

 
ament_target_dependencies(utility_lib PUBLIC rclcpp addverb_cobot_msgs)
target_link_libraries(utility_lib PUBLIC ${COBOT_UI_BACKEND_LIBRARIES})
target_include_directories(utility_lib
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include
  ${COBOT_UI_BACKEND_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

install(
    DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS utility_lib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(addverb_cobot_msgs)


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
