# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

ROS2 package for Intel RealSense camera service with hand-eye calibration capabilities. Provides both service-based image capture and continuous streaming for vision-guided robotic manipulation.

## Build System

This package uses **ament_cmake** (not ament_python) despite being primarily Python code. This hybrid approach is required because:
- Custom service interfaces need `rosidl_generate_interfaces`
- Using `${PROJECT_NAME}` as the target name for both service generation and Python package installation causes conflicts
- Solution: Manual Python installation via CMake `install()` commands with symlinks

### Build Commands

```bash
# Build package
cd ~/ws
source /opt/ros/humble/setup.bash
colcon build --packages-select realsense_service

# Source after build
source install/setup.bash

# Clean build (if needed)
rm -rf build install log
colcon build --packages-select realsense_service
```

### Installing Dependencies

```bash
# System dependencies
sudo apt-get install -y ros-humble-librealsense2* ros-humble-cv-bridge

# Python dependencies
pip3 install pyrealsense2 opencv-contrib-python numpy pyyaml
```

## Architecture

### Service-Based Camera Control

**Main Node**: `realsense_service_node.py`
- Manages RealSense pipeline lifecycle
- Dual mode: on-demand capture (service) + continuous streaming (topics)
- Default resolution: 848x480 @ 30fps (optimized for D405)
- Services: `/capture_image`, `/set_camera_state`
- Topics: `~/color/image_raw`, `~/depth/image_raw`

### Hand-Eye Calibration System

Three-node architecture for robot-mounted camera calibration:

1. **`hand_eye_calibration_node.py`**: Core calibration logic
   - Detects ArUco markers or checkerboards in camera view
   - Collects robot end-effector poses
   - Computes camera-to-gripper transform using OpenCV
   - Supports 3 robot pose sources: **TF (default/recommended)**, topic, joint_states
   - Services: `/capture_calibration_sample`, `/compute_calibration`, `/reset_calibration`

2. **`calibration_helper.py`**: Interactive data collection UI
   - Keyboard interface: SPACE=capture, C=compute, R=reset, Q=quit
   - Service client for calibration operations
   - Terminal-based user interaction

3. **`robot_pose_publisher.py`**: Test robot simulator
   - Only for testing without real robot hardware
   - Publishes dummy poses via topic or TF

**Critical Design Note**: The calibration system primarily uses **TF** to get robot poses (`base_link` -> `tool0`), not topics. This matches real robot systems that broadcast TF. The topic-based mode exists for legacy/special cases only.

### Calibration Result Integration

After calibration, three tools for using the camera-to-gripper transform:

1. **`camera_tf_broadcaster.py`**: Broadcasts calibration as TF at runtime
   - Loads YAML calibration file (auto-detects latest)
   - Publishes `camera_link` and optical frames as TF
   - Production-ready for vision-guided manipulation

2. **URDF Integration**: `urdf/realsense_d405.xacro`
   - XACRO macro for permanently adding camera to robot model
   - Includes physical properties (42x28x22mm, 32g)
   - Creates all required frames (camera_link, color/depth optical frames)

3. **Vision Application Example**: `examples/vision_guided_pick.py`
   - Complete pick-and-place workflow
   - Demonstrates pixel → 3D → robot coordinate transformation
   - Uses TF for coordinate frame conversion

## Custom Service Interfaces

Located in `srv/` directory:

**CaptureImage.srv**:
```
bool enable_color
bool enable_depth
int32 width   # 0 = use default 848
int32 height  # 0 = use default 480
---
bool success
string message
sensor_msgs/Image color_image
sensor_msgs/Image depth_image
```

**SetCameraState.srv**:
```
bool start  # true=start, false=stop
---
bool success
string message
```

Generated interfaces available after build: `from realsense_service.srv import CaptureImage, SetCameraState`

## Common Workflows

### Running Camera Service

```bash
# Basic streaming
ros2 run realsense_service realsense_service_node

# With RViz
ros2 launch realsense_service realsense_with_rviz.launch.py

# Test capture
ros2 service call /capture_image realsense_service/srv/CaptureImage \
  "{enable_color: true, enable_depth: true, width: 848, height: 480}"
```

### Hand-Eye Calibration

```bash
# Terminal 1: Launch calibration system (using TF for robot pose)
ros2 launch realsense_service hand_eye_calibration.launch.py \
  pose_source:=tf \
  robot_base_frame:=base_link \
  robot_ee_frame:=tool0 \
  marker_type:=aruco \
  marker_size:=0.05

# Terminal 2: Interactive helper
ros2 run realsense_service calibration_helper

# Move robot to various poses, press SPACE to capture samples
# After 10-20 samples, press C to compute calibration
# Results saved to ~/calibration_data/hand_eye_calibration_*.yaml
```

### Using Calibration Results

```bash
# Method 1: Runtime TF broadcaster (quick testing)
ros2 run realsense_service camera_tf_broadcaster

# Method 2: Add to robot URDF (see urdf/robot_with_camera_example.urdf.xacro)
# Method 3: Static TF publisher (simple testing)
ros2 run tf2_ros static_transform_publisher --x 0.05 --y 0.02 --z 0.08 ...
```

## Important Files

- `CMakeLists.txt`: Hybrid build system with service generation + manual Python installation
- `package.xml`: Build dependencies (note: ament_cmake not ament_python)
- `CALIBRATION_RESULT_USAGE.md`: Complete guide for using calibration results in production
- `HAND_EYE_CALIBRATION_GUIDE.md`: Step-by-step calibration procedures

## Critical Implementation Details

### CMakeLists.txt Pattern

DO NOT use `ament_python_install_package()` alongside `rosidl_generate_interfaces()` with the same project name. Instead:

```cmake
# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CaptureImage.srv"
  "srv/SetCameraState.srv"
  DEPENDENCIES sensor_msgs
)

# Manual Python installation
install(DIRECTORY ${PROJECT_NAME}/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  PATTERN "*.pyc" EXCLUDE
  PATTERN "__pycache__" EXCLUDE
)

# Install executables with symlinks (ROS2 expects no .py extension)
install(PROGRAMS realsense_service/realsense_service_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(CODE "execute_process(
  COMMAND ${CMAKE_COMMAND} -E create_symlink
    realsense_service_node.py
    \${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/realsense_service_node
)")
```

### Robot Pose Acquisition

The calibration node supports three modes via `pose_source` parameter:

1. **`tf`** (default): Uses TF2 to look up `base_link` → `tool0` transform
   - Most common in real robot systems
   - Requires robot driver to broadcast TF (most do)
   - Parameters: `robot_base_frame`, `robot_ee_frame`

2. **`topic`**: Subscribes to `PoseStamped` topic
   - Legacy mode for systems that publish pose directly
   - Parameter: `robot_pose_topic`

3. **`joint_states`**: Uses joint angles with forward kinematics
   - Not implemented (marked as TODO)
   - Would need robot-specific FK implementation

### Camera Resolution

Default is **848x480** (not 640x480 or 1280x720) because:
- RealSense D405 is optimized for this resolution
- Good balance of quality and performance
- Native resolution for depth + color alignment

## Testing

```bash
# Check services are running
ros2 service list | grep -E "(capture_image|set_camera_state|calibration)"

# Verify topics are publishing
ros2 topic hz /realsense_service_node/color/image_raw

# Test camera connection
realsense-viewer  # Outside ROS2, direct SDK test
lsusb | grep Intel

# Check TF tree
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo base_link tool0
```

## Target Hardware

- **Camera**: Intel RealSense D405 (7cm-4m range, 42x28x22mm, 32g)
- **Robot**: Any ROS2-compatible robot arm broadcasting TF
- **Calibration Target**: ArUco marker (DICT_6X6_250, 5cm) or checkerboard (9x6, 2.5cm squares)
- **OS**: Ubuntu 22.04 with ROS2 Humble

## Language Note

Documentation and code comments are in Korean (한국어) as per user preference. Code identifiers, commit messages, and technical terms use English.
