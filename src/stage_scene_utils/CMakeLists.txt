cmake_minimum_required(VERSION 3.8)
project(stage_scene_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_srvs REQUIRED)

# Executable: add_stage_to_scene
add_executable(add_stage_to_scene src/add_stage_to_scene.cpp)

ament_target_dependencies(add_stage_to_scene
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  shape_msgs
  geometric_shapes
  tf2
  tf2_geometry_msgs
)

target_include_directories(add_stage_to_scene PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)

# C++17 for std::filesystem
target_compile_features(add_stage_to_scene PRIVATE cxx_std_17)

# Executable: plan_with_upright
add_executable(plan_with_upright src/plan_with_upright.cpp)

ament_target_dependencies(plan_with_upright
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  std_srvs
)

target_compile_features(plan_with_upright PRIVATE cxx_std_17)

# Install executables
install(TARGETS
  add_stage_to_scene
  plan_with_upright
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/move_relative.py
  scripts/remove_stage.py
  scripts/check_acm.py
  scripts/list_scene.py
  scripts/reset_acm.py
  scripts/test_collision.py
  scripts/plan_with_orientation_constraint.py
  scripts/add_joint_limits.py
  scripts/plan_upright_pymoveit2.py
  scripts/set_upright_constraint.py
  scripts/check_constraints.py
  scripts/set_scene_constraints.py
  scripts/clear_constraints.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
