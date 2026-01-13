cmake_minimum_required(VERSION 3.8)
project(epics_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(yaml-cpp REQUIRED)

# Find EPICS CA library
# EPICS_BASE environment variable should point to EPICS base installation
if(DEFINED ENV{EPICS_BASE})
  set(EPICS_BASE $ENV{EPICS_BASE})
else()
  set(EPICS_BASE "/usr/local/epics/base")
endif()

# Determine EPICS host architecture
if(DEFINED ENV{EPICS_HOST_ARCH})
  set(EPICS_HOST_ARCH $ENV{EPICS_HOST_ARCH})
else()
  # Default for Linux x86_64
  set(EPICS_HOST_ARCH "linux-x86_64")
endif()

set(EPICS_INCLUDE_DIR "${EPICS_BASE}/include")
set(EPICS_OS_INCLUDE_DIR "${EPICS_BASE}/include/os/Linux")
set(EPICS_COMPILER_INCLUDE_DIR "${EPICS_BASE}/include/compiler/gcc")
set(EPICS_LIB_DIR "${EPICS_BASE}/lib/${EPICS_HOST_ARCH}")

# Check if EPICS is available
if(EXISTS "${EPICS_INCLUDE_DIR}/cadef.h")
  set(EPICS_FOUND TRUE)
  message(STATUS "Found EPICS CA at: ${EPICS_BASE}")
  message(STATUS "EPICS host arch: ${EPICS_HOST_ARCH}")
else()
  set(EPICS_FOUND FALSE)
  message(WARNING "EPICS CA not found at ${EPICS_BASE}. epics_triggered_sequence will not be built.")
endif()

add_executable(mtc_node src/mtc_node.cpp)
target_include_directories(mtc_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(mtc_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  mtc_node
  "moveit_task_constructor_core"
  "rclcpp"
)

add_executable(pick_and_place_task src/pick_and_place_task.cpp)
target_include_directories(pick_and_place_task PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(pick_and_place_task PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  pick_and_place_task
  "moveit_task_constructor_core"
  "rclcpp"
  "geometry_msgs"
)

add_executable(move_relative_mtc src/move_relative_mtc.cpp)
target_include_directories(move_relative_mtc PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(move_relative_mtc PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  move_relative_mtc
  "moveit_task_constructor_core"
  "rclcpp"
  "geometry_msgs"
  "std_msgs"
  "rcl_interfaces"
)

add_executable(sample_load_sequence src/sample_load_sequence.cpp)
target_include_directories(sample_load_sequence PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sample_load_sequence PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  sample_load_sequence
  "moveit_task_constructor_core"
  "rclcpp"
  "rclcpp_action"
  "geometry_msgs"
  "control_msgs"
  "std_msgs"
  "std_srvs"
  "rcl_interfaces"
)

add_executable(multi_holder_sequence src/multi_holder_sequence.cpp)
target_include_directories(multi_holder_sequence PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(multi_holder_sequence PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  multi_holder_sequence
  "moveit_task_constructor_core"
  "moveit_ros_planning_interface"
  "rclcpp"
  "rclcpp_action"
  "geometry_msgs"
  "control_msgs"
  "std_msgs"
  "std_srvs"
  "rcl_interfaces"
)

# epics_triggered_sequence - only build if EPICS CA is available
if(EPICS_FOUND)
  add_executable(epics_triggered_sequence src/epics_triggered_sequence.cpp)
  target_include_directories(epics_triggered_sequence PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EPICS_INCLUDE_DIR}
    ${EPICS_OS_INCLUDE_DIR}
    ${EPICS_COMPILER_INCLUDE_DIR}
  )
  target_link_directories(epics_triggered_sequence PUBLIC ${EPICS_LIB_DIR})
  target_compile_features(epics_triggered_sequence PUBLIC c_std_99 cxx_std_17)
  ament_target_dependencies(
    epics_triggered_sequence
    "moveit_task_constructor_core"
    "moveit_ros_planning_interface"
    "rclcpp"
    "rclcpp_action"
    "geometry_msgs"
    "control_msgs"
    "std_msgs"
    "std_srvs"
    "rcl_interfaces"
  )
  # Link EPICS CA libraries and yaml-cpp
  target_link_libraries(epics_triggered_sequence ca Com yaml-cpp)
endif()

add_executable(upright_constraint_test src/upright_constraint_test.cpp)
target_include_directories(upright_constraint_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(upright_constraint_test PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  upright_constraint_test
  "moveit_task_constructor_core"
  "rclcpp"
)

add_executable(add_stage_to_scene src/add_stage_to_scene.cpp)
target_include_directories(add_stage_to_scene PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(add_stage_to_scene PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  add_stage_to_scene
  "rclcpp"
  "moveit_ros_planning_interface"
  "geometric_shapes"
)

install(TARGETS mtc_node add_stage_to_scene
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS pick_and_place_task
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS move_relative_mtc sample_load_sequence multi_holder_sequence upright_constraint_test
  DESTINATION lib/${PROJECT_NAME})

if(EPICS_FOUND)
  install(TARGETS epics_triggered_sequence
    DESTINATION lib/${PROJECT_NAME})
endif()

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

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/add_stage_to_scene.py
  scripts/ur3e_hande_pickplace.py
  scripts/setup_stage_acm.py
  scripts/fix_acm.py
  DESTINATION lib/${PROJECT_NAME}
)