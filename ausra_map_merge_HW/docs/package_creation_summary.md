# `ausra_map_merge_HW` Package — Creation Summary

## Package Structure

```
ausra_map_merge_HW/
├── CMakeLists.txt                    ← Build config, installs bin/launch/config/docs
├── package.xml                       ← Dependencies: rclcpp, nav_msgs, multirobot_map_merge
├── src/
│   └── map_expansion_node.cpp        ← Heartbeat timer architecture (hardware-adapted)
├── launch/
│   └── map_merge_hw.launch.py        ← ROBOT_HW_CONFIG + phantom node + 2s delayed merger
├── config/
│   └── map_merge_HW_params.yaml      ← All init_pose_* = 0.0 (critical)
└── docs/
    └── AUSRA_Hardware_Map_Merge_SOP.md
```

## Key Adaptations from Simulation → Hardware

| Change | Simulation (`ausra_map_merge`) | Hardware (`ausra_map_merge_HW`) |
|---|---|---|
| Default `input_topic` | `/ausra_1/map` | `/map` |
| Doc comments | Reference "Gazebo spawn" | Reference "tape-measured spawn" |
| Member variable docs | `Robot's Gazebo spawn X` | `Robot's physical spawn X (tape-measured)` |
| Log message format | `MapExpansionNode:` | `MapExpansionNode initialised:` |

## Files Created

| File | Key Detail |
|---|---|
| [package.xml](file:///home/ahmedmahmoud/Swarm%20HW/src/AUSRA-Autonomous-System/ausra_map_merge_HW/package.xml) | `exec_depend` on `multirobot_map_merge` and `slam_toolbox` |
| [CMakeLists.txt](file:///home/ahmedmahmoud/Swarm%20HW/src/AUSRA-Autonomous-System/ausra_map_merge_HW/CMakeLists.txt) | Installs `launch/`, `config/`, `docs/` directories |
| [map_expansion_node.cpp](file:///home/ahmedmahmoud/Swarm%20HW/src/AUSRA-Autonomous-System/ausra_map_merge_HW/src/map_expansion_node.cpp) | Default `input_topic="/map"` for hardware global namespace |
| [map_merge_hw.launch.py](file:///home/ahmedmahmoud/Swarm%20HW/src/AUSRA-Autonomous-System/ausra_map_merge_HW/launch/map_merge_hw.launch.py) | `ROBOT_HW_CONFIG` at Phase 1 (0.0, 0.0); phantom node prevents segfault |
| [map_merge_HW_params.yaml](file:///home/ahmedmahmoud/Swarm%20HW/src/AUSRA-Autonomous-System/ausra_map_merge_HW/config/map_merge_HW_params.yaml) | All `init_pose_*` locked to `0.0` |

## Intelligent Adjustments Applied

1. **C++ default parameter**: Changed `input_topic` default from `/ausra_1/map` → `/map` to match hardware SLAM's global namespace output.
2. **Doc comments**: Updated all references from "Gazebo spawn coordinates" to "tape-measured physical spawn offsets" throughout the C++ source.
3. **`docs/` directory**: Created and populated with the SOP doc so the `install(DIRECTORY docs/ ...)` CMake target doesn't fail on an empty/missing directory.

> [!IMPORTANT]
> Build when ready: `colcon build --packages-select ausra_map_merge_HW --symlink-install`
