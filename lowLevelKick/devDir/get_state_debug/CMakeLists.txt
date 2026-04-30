cmake_minimum_required(VERSION 3.5)
project(get_state_debug)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)
include_directories(/opt/unitree_robotics/include)
include_directories(/opt/unitree_robotics/include/ddscxx)

link_directories(/opt/unitree_robotics/lib)  
link_directories(src)

set (
  DEPENDENCY_LIST
  unitree_go
  rclcpp
  std_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

add_executable(get_state_debug src/get_state_debug.cpp src/motor_crc.cpp)

ament_target_dependencies(get_state_debug ${DEPENDENCY_LIST})
target_link_libraries(get_state_debug unitree_sdk2 ddsc ddscxx pthread)

set_target_properties(get_state_debug PROPERTIES
    INSTALL_RPATH "/opt/unitree_robotics/lib"
    BUILD_WITH_INSTALL_RPATH TRUE
)

install(TARGETS 
        get_state_debug
        ${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
