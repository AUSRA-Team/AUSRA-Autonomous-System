# AUSRA Exploration Maps
This directory contains maps saved by the frontier exploration system.

## Map Files
Each map exploration session saves:
- `<name>.pgm` - The occupancy grid image (grayscale)
- `<name>.yaml` - Map metadata (resolution, origin, etc.)
- `<name>.posegraph` - SLAM Toolbox pose graph (for localization continuation)

## Usage

### Load a saved map for localization:
```bash
ros2 launch ausra_spawner robot_nav2_test.launch.py \
    robot_id:=1 \
    use_slam:=false \
    map_file:=/path/to/map.yaml
```

### Continue SLAM from saved pose graph:
```bash
# Start SLAM in localization mode with pose graph
ros2 launch ausra_spawner robot_bringup.launch.py \
    robot_id:=1 \
    slam_mode:=localization \
    map_file:=/path/to/map.posegraph
```
