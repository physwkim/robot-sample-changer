cmake_minimum_required(VERSION 3.8)
project(realsense_service)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclpy REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CaptureImage.srv"
  "srv/SetCameraState.srv"
  DEPENDENCIES sensor_msgs
)

# Python install path
if(NOT PYTHON_INSTALL_DIR)
  set(PYTHON_INSTALL_DIR "lib/python3/dist-packages")
endif()

# Install Python modules
install(DIRECTORY ${PROJECT_NAME}/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

# Install Python executables
install(PROGRAMS
  realsense_service/realsense_service_node.py
  realsense_service/hand_eye_calibration_node.py
  realsense_service/calibration_helper.py
  realsense_service/camera_tf_broadcaster.py
  examples/update_planning_scene.py
  DESTINATION lib/${PROJECT_NAME}
)

# Create symlinks without .py extension for ROS2 to find the executables
install(CODE "execute_process(
  COMMAND ${CMAKE_COMMAND} -E create_symlink
    realsense_service_node.py
    \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/realsense_service_node
)")

install(CODE "execute_process(
  COMMAND ${CMAKE_COMMAND} -E create_symlink
    hand_eye_calibration_node.py
    \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/hand_eye_calibration_node
)")

install(CODE "execute_process(
  COMMAND ${CMAKE_COMMAND} -E create_symlink
    calibration_helper.py
    \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/calibration_helper
)")

install(CODE "execute_process(
  COMMAND ${CMAKE_COMMAND} -E create_symlink
    camera_tf_broadcaster.py
    \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/camera_tf_broadcaster
)")

install(CODE "execute_process(
  COMMAND ${CMAKE_COMMAND} -E create_symlink
    update_planning_scene.py
    \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/update_planning_scene
)")

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install RViz config files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install URDF files
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
