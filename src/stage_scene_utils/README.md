# Stage Scene Utils

Utilities for adding stage mesh to MoveIt planning scene.

## Overview

This package provides a simple executable to add STL mesh files to the MoveIt planning scene. This is useful for adding static collision objects like stages, tables, or other environmental fixtures.

## Building

```bash
cd /home/stevek/ws
source /opt/ros/humble/setup.bash
colcon build --packages-select stage_scene_utils --symlink-install
```

## Usage

### Basic Usage

```bash
# Source the workspace
source /home/stevek/ws/install/setup.bash

# Run with default parameters (assumes /home/stevek/ws/stage.stl exists)
ros2 run stage_scene_utils add_stage_to_scene
```

### Custom Parameters

```bash
# Custom STL path
ros2 run stage_scene_utils add_stage_to_scene \
  --ros-args \
  -p stl_path:=/path/to/your/stage.stl

# Custom position (e.g., lower the stage)
ros2 run stage_scene_utils add_stage_to_scene \
  --ros-args \
  -p position:=[0.0,0.0,-0.05]

# Custom scale (e.g., convert mm to m)
ros2 run stage_scene_utils add_stage_to_scene \
  --ros-args \
  -p scale:=[0.001,0.001,0.001]

# All parameters
ros2 run stage_scene_utils add_stage_to_scene \
  --ros-args \
  -p stl_path:=/home/stevek/ws/stage.stl \
  -p object_name:=stage \
  -p frame_id:=world \
  -p position:=[0.0,0.0,-0.05] \
  -p scale:=[0.001,0.001,0.001]
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `stl_path` | string | `/home/stevek/ws/stage.stl` | Path to STL mesh file |
| `object_name` | string | `stage` | Name of collision object in planning scene |
| `frame_id` | string | `world` | Reference frame for the mesh |
| `position` | double[] | `[0.0, 0.0, 0.0]` | XYZ position of mesh origin |
| `scale` | double[] | `[1.0, 1.0, 1.0]` | XYZ scale factors |

## Complete Example

### Terminal 1: Launch MoveIt

```bash
source /home/stevek/ws/install/setup.bash

ros2 launch ur3e_hande_moveit_config ur_moveit.launch.py \
  ur_type:=ur3e \
  launch_rviz:=true \
  description_package:=ur3e_hande_robot_description \
  description_file:=ur_with_hande.xacro \
  moveit_config_package:=ur3e_hande_moveit_config \
  moveit_config_file:=ur.srdf
```

### Terminal 2: Add Stage Mesh

```bash
source /home/stevek/ws/install/setup.bash

ros2 run stage_scene_utils add_stage_to_scene
```

### RViz Visualization

In RViz:
1. Ensure `PlanningScene` display is added
2. Enable `Scene Display` → `Scene Geometry`
3. Adjust `Scene Alpha` for transparency (0.5-0.9)
4. The stage mesh should appear in the `world` frame

## Troubleshooting

### Mesh not visible in RViz
- Check that MoveIt is fully initialized before running
- Verify the STL file path exists
- Check RViz display settings (Scene Geometry enabled)
- Try adjusting scale if mesh is too large/small

### Scale issues
If the mesh appears very large or very small, your CAD file may use different units:
- **mm units**: Use `scale:=[0.001,0.001,0.001]`
- **cm units**: Use `scale:=[0.01,0.01,0.01]`
- **inches**: Use `scale:=[0.0254,0.0254,0.0254]`

### Position adjustment
If the mesh is floating or underground:
- Adjust Z position: `position:=[0.0,0.0,-0.05]`
- Check your STL file's origin in the CAD software

## Integration with pdf_beamtime

This utility complements the existing `pdf_beamtime` obstacle management:
- `pdf_beamtime` uses YAML-defined primitive shapes (boxes, cylinders)
- `stage_scene_utils` adds complex mesh geometries from STL files

Both approaches update the same MoveIt planning scene, so they can be used together.

## License

BSD 3-Clause License. See LICENSE.txt for details.
