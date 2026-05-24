# Decentralized Map Merge: Code Changes Explained
**Document:** `Decentralized_Changes_Explained.md`

Moving to a fully decentralized swarm architecture means every robot runs its own map merger. To do this without creating duplicate files for every robot, we had to make our launch and config files "self-aware" and dynamic. 

Here is the exact breakdown of what changed in your code and why.

---

## 1. Changes to `map_merge_HW_params.yaml`

In the centralized setup, the map merger ran in the global namespace. Now, it runs inside the robot's specific namespace (e.g., `/ausra_1/map_merge`). We had to update the configuration file so it applies to *any* robot that reads it.

### The Wildcard Target
**Before:**
```yaml
map_merge:
  ros__parameters:
  After:

YAML
/**/map_merge:
  ros__parameters:
Why: The // is a ROS 2 wildcard. It tells the parameter server: "Apply these settings to any node named map_merge, regardless of what namespace it is inside." This allows /ausra_1/map_merge and /ausra_2/map_merge to share the exact same YAML file.

Relative Output Topic
Before:

YAML
    merged_map_topic:  /map_merged
After:

YAML
    merged_map_topic:  map_merged
Why: By removing the leading slash /, the topic becomes relative. When Robot 1 runs the merger, the output automatically resolves to /ausra_1/map_merged. This prevents Robot 1 and Robot 2 from publishing their merged maps to the same global topic.

2. Changes to map_merge_hw.launch.py
The launch file went from a static script that launched everything at once, to a dynamic script that configures itself based on which robot is running it.

Addition of Launch Arguments
The Change:

Python
DeclareLaunchArgument('robot_name', default_value='ausra_1')
Why: We added a launch argument so the user can tell the script which robot it is running on via the terminal (e.g., ros2 launch ... robot_name:=ausra_2).

The OpaqueFunction
The Change:

Python
def generate_decentralized_nodes(context):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    # ... node setup continues ...
Why: In ROS 2, LaunchConfiguration variables are usually locked until the launch file actually executes. An OpaqueFunction allows us to "peek" at the robot_name argument before the nodes are built. This lets us use the robot_name string to dynamically pull the correct offsets from the ROBOT_HW_CONFIG dictionary.

The Local Phantom Node
The Change: We re-introduced a phantom node, but this time it is specifically named map_expansion_<robot_name>_phantom.
Why: In a decentralized swarm, Robot 1 might boot up 30 seconds before Robot 2. If Robot 1's local map merger starts up and only sees one map, it will crash (exit code -11). The local phantom ensures Robot 1's merger always sees at least two maps, keeping it alive until teammates join the network.

Namespacing the Merger
The Change:

Python
Node(
    package='multirobot_map_merge',
    executable='map_merge',
    name='map_merge',
    namespace=f'/{robot_name}',   # <--- The critical change
    parameters=[params_file],
)
Why: Pushing the map_merge node into the /{robot_name} namespace ensures that it does not conflict with other robots on the Wi-Fi network. It isolates the computation pipeline locally to the robot.