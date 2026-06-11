import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, 
                            TimerAction, LogInfo, OpaqueFunction, ExecuteProcess,
                            GroupAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml

def generate_nav2_config(robot_name: str, base_config_path: str) -> str:
    """Generate Nav2 config with robot-specific frame names, written to a temp file."""
    with open(base_config_path, 'r') as f:
        content = f.read()
    
    content = content.replace('<robot_namespace>', robot_name)
    
    config_dir = tempfile.mkdtemp(prefix='ausra_nav2_')
    config_path = os.path.join(config_dir, f'{robot_name}_nav2.yaml')
    with open(config_path, 'w') as f:
        f.write(content)
    
    return config_path

def generate_hardware_params(robot_name: str, base_config_path: str) -> str:
    """Rewrite <robot_namespace> placeholders in hardware_params.yaml."""
    with open(base_config_path, 'r') as f:
        content = f.read()

    content = content.replace('<robot_namespace>', robot_name)

    config_dir = tempfile.mkdtemp(prefix='ausra_hw_')
    config_path = os.path.join(config_dir, f'{robot_name}_hardware_params.yaml')
    with open(config_path, 'w') as f:
        f.write(content)

    return config_path

def generate_full_hardware_stack(context):
    # ── Resolve robot_name at context time ────────────────────────────────
    robot_name = LaunchConfiguration('robot_name').perform(context)

    # ── Dynamically generated TF frame names ──────────────────────────────
    base_frame = f'{robot_name}_robot_footprint'
    odom_frame = f'{robot_name}_odom'
    lidar_frame = f'{robot_name}_lidar'
    imu_frame  = f'{robot_name}_imu_link'
    map_frame  = f'{robot_name}_map'

    # ── Package directories ───────────────────────────────────────────────
    pkg_lidar_slam  = get_package_share_directory('lidar_slam_pkg')
    pkg_localization = get_package_share_directory('ausra_localization')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    pkg_description  = get_package_share_directory('ausrabot_description')
    pkg_lidar        = get_package_share_directory('sllidar_ros2')
    pkg_imu          = get_package_share_directory('mpu6050driver')

    # ── Config paths ──────────────────────────────────────────────────────
    nav2_params_file   = os.path.join(pkg_lidar_slam, 'config', 'nav2_holonomic_params.yaml')
    slam_config_file   = os.path.join(pkg_lidar_slam, 'config', 'slam_toolbox_config.yaml')
    # hardware_params    = os.path.join(pkg_description, 'config', 'hardware_params.yaml')
    xacro_file         = os.path.join(pkg_description, 'urdf', 'robot.urdf.xacro')
    explore_params_file = os.path.join(pkg_lidar_slam, 'config', 'explore_params.yaml')
    ekf_params_file    = os.path.join(pkg_localization, 'config', 'ekf.yaml')
    imu_params_file    = os.path.join(pkg_localization, 'config', 'imu_complimentary_filter.yaml')
    mpu6050_params_file = os.path.join(pkg_imu, 'params', 'mpu6050.yaml')

    # ── Launch Configurations ─────────────────────────────────────────────
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    nudge_robot  = LaunchConfiguration('nudge_robot', default='false')

    # Get spawn position
    x = LaunchConfiguration('x', default='0.0').perform(context)
    y = LaunchConfiguration('y', default='0.0').perform(context)
    yaw = LaunchConfiguration('yaw', default='0.0').perform(context)

    # ══════════════════════════════════════════════════════════════════════
    # Stage 0: Core Hardware & Description
    # ══════════════════════════════════════════════════════════════════════

    # Pass robot_name into xacro so all URDF links become <robot_name>_*
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' robot_name:=', robot_name]),
        value_type=str
    )

    send_ns = ExecuteProcess(
        cmd=['bash', '-c', f'stty -F /dev/ttyACM0 6000000 && echo "{robot_name}" > /dev/ttyACM0'],
        name='send_namespace',
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'ignore_timestamp': True,
        }],
    )

    # Override frame IDs and joint names so the omni driver publishes
    # TF for the correct prefixed frames.
    hardware_params_base = os.path.join(pkg_description, 'config', 'hardware_params.yaml')
    hardware_params = generate_hardware_params(robot_name, hardware_params_base)
    omni_driver = Node(
        package='omnidirectional_driver',
        executable='omni_driver',
        name='omnidirectional_driver',
        output='screen',
        parameters=[hardware_params, {
            'use_sim_time': use_sim_time,
            'odom_frame_id': odom_frame,
            'base_frame_id': base_frame,
        }],
        # Ensure the driver uses namespace-relative topics.
        # If the driver internally publishes to "/odom" or "/cmd_vel",
        # these remappings force them into the namespace.
        remappings=[
            ('/odom', 'odom'),
            ('/cmd_vel', 'cmd_vel'),
            ('/joint_states', 'joint_states'),
            ('/joint_group_velocity_controller/commands', 'joint_group_velocity_controller/commands'),
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ],
    )

    lidar_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_lidar, 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': '115200',
            'frame_id': lidar_frame,
        }.items()
    )

    # MPU6050 Raw IMU Driver — publishes sensor_msgs/Imu on relative 'imu'
    # topic. The frame_id parameter is injected so the Imu header matches
    # the prefixed TF tree.
    mpu6050_driver = Node(
        package='mpu6050driver',
        executable='mpu6050driver',
        name='mpu6050publisher',
        output='screen',
        emulate_tty=True,
        parameters=[mpu6050_params_file, {
            'frame_id': imu_frame,   # e.g. ausra_1_imu_link
        }],
    )

    # ESP32 Micro-ROS Agent — must use ExecuteProcess because the
    # micro_ros_agent binary expects CLI arguments, not ROS Node parameters.
    # ExecuteProcess is NOT affected by PushRosNamespace, so we inject
    # the namespace explicitly via --ros-args -r __ns:=.
    micro_ros_agent = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'micro_ros_agent', 'micro_ros_agent',
            'serial', '--dev', '/dev/ttyACM0',
            '--ros-args', '-r', f'__ns:=/{robot_name}',
        ],
        output='screen',
    )

    # ══════════════════════════════════════════════════════════════════════
    # Stage 1: Localization (IMU Filter + EKF) + SLAM
    # ══════════════════════════════════════════════════════════════════════

    # IMU Complementary Filter — fuses raw accel + gyro into a filtered
    # orientation.  The fixed_frame and output frame_id are overridden so
    # the published sensor_msgs/Imu header.frame_id matches the prefixed
    # TF tree.
    # imu_filter_node = Node(
    #     package='imu_complementary_filter',
    #     executable='complementary_filter_node',
    #     name='complementary_filter_gain_node',
    #     output='screen',
    #     parameters=[imu_params_file, {
    #         'use_sim_time': use_sim_time,
    #         'fixed_frame': odom_frame,          # was: ausrabot_odom
    #     }],
    #     remappings=[
    #         # Force raw-IMU input and filtered output into the namespace.
    #         # If the driver publishes to absolute /imu/data_raw, this
    #         # remapping catches it.
    #         ('/imu/data_raw', 'imu/data_raw'),
    #         ('/imu/data',     'imu/data'),
    #         ('/imu/mag',      'imu/mag'),
    #         # Output filtered data on a relative topic the EKF can find.
    #         ('/imu',          'imu'),
    #     ],
    # )
    map_offset_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_offset_publisher',
        arguments=[
            x, y, '0.0', yaw, '0.0', '0.0',  # x y z yaw pitch roll
            'map', map_frame,
        ],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ],
        output='screen'
    )

    # Inline EKF node — inject prefixed frame names and relative topics.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_params_file, {
            'use_sim_time': use_sim_time,
            'odom_frame': odom_frame,
            'base_link_frame': base_frame,
            'world_frame': odom_frame,
            'map_frame': map_frame,
            # Relative topics — PushRosNamespace will prepend /<robot_name>/.
            'odom0': 'odom',
            'imu0': 'imu',
        }],
        remappings=[
            ('odometry/filtered', 'filtered_odometry'),
        ],
    )

    # SLAM Toolbox — override frames and use relative scan topic.
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_file,
            {
                'use_sim_time': use_sim_time,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'map_frame': map_frame,
                # CRITICAL: relative topic so PushRosNamespace applies.
                'scan_topic': 'scan',
            }
        ],
        remappings=[
            # Slam Toolbox expects absolute /scan topic by default, but we remap it to relative 'scan' so the namespace is applied.
            ('/scan', 'scan'),
            ('/map', 'map'),
        ],
    )

    # ══════════════════════════════════════════════════════════════════════
    # Stage 2: Nav2 Navigation
    # ══════════════════════════════════════════════════════════════════════

    # Dynamically rewrite the Nav2 YAML file to inject the prefixed frame names
    # (e.g. ausra_1_robot_footprint) without requiring manual sed hacks.
    #
    # NOTE: 'global_frame' is intentionally EXCLUDED from RewrittenYaml.
    # RewrittenYaml rewrites ALL occurrences of a key name globally. Both
    # the global_costmap (needs 'map') and local_costmap (needs odom_frame)
    # share a key called 'global_frame', so a blanket rewrite would corrupt
    # one of them. Instead, we use RewrittenYaml for the safe keys, then
    # selectively patch local_costmap.global_frame via a second pass.
    param_substitutions = {
        'robot_base_frame': base_frame,
        'base_frame_id': base_frame,
        'odom_frame_id': odom_frame,
        'local_frame': odom_frame,
    }

    configured_nav2_params = RewrittenYaml(
        source_file=nav2_params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True
    )

    nav2_nodes = []

    nav2_config_base = os.path.join(pkg_lidar_slam, 'config', 'nav2_holonomic_params.yaml')
        
    # Use YAML-aware config loading to preserve structure including DWB critics
    nav2_config = generate_nav2_config(robot_name, nav2_config_base)
    
    # Nav2 Lifecycle Manager
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]
    
    nav2_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        namespace=robot_name,
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': lifecycle_nodes,
            'bond_timeout': 10.0,
            'attempt_respawn_reconnection': True,
            'bond_respawn_max_duration': 10.0,
        }]
    )
    
    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        namespace=robot_name,
        output='screen',
        parameters=[nav2_config],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )
    
    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        namespace=robot_name,
        output='screen',
        parameters=[nav2_config],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )
    
    # Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        namespace=robot_name,
        output='screen',
        parameters=[nav2_config],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )
    
    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        namespace=robot_name,
        output='screen',
        parameters=[nav2_config],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )
    
    # Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        namespace=robot_name,
        output='screen',
        parameters=[nav2_config],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
        ]
    )
    
    # Velocity Smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        namespace=robot_name,
        output='screen',
        parameters=[nav2_config],
        remappings=[
            ('/tf', '/tf'),
            ('/tf_static', '/tf_static'),
            ('cmd_vel', 'cmd_vel_nav'),
            ('cmd_vel_smoothed', 'cmd_vel'),
        ]
    )

    nav2_nodes.extend([
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        nav2_lifecycle_manager,
    ])

    # ══════════════════════════════════════════════════════════════════════
    # Stage 3: Exploration
    # ══════════════════════════════════════════════════════════════════════

    exploration_server = Node(
        package='explore_lite',
        name='explore_node',
        executable='explore',
        parameters=[explore_params_file, {
            'use_sim_time': False,
            'robot_base_frame': base_frame,
            # Override absolute topics from YAML with relative ones.
            'costmap_topic': 'global_costmap/costmap',
            'costmap_updates_topic': 'global_costmap/costmap_updates',
        }],
        output='screen',
    )

    # ══════════════════════════════════════════════════════════════════════
    # Assemble — Wrap EVERYTHING in GroupAction + PushRosNamespace
    # ══════════════════════════════════════════════════════════════════════
    #
    # CRITICAL: In ROS 2 Humble, TimerAction does NOT propagate
    # PushRosNamespace to its children. Every delayed stage MUST re-push
    # the namespace inside its own GroupAction to ensure all nodes and
    # IncludeLaunchDescriptions inherit the correct /<robot_name>/ prefix.
    #
    # micro_ros_agent uses ExecuteProcess (immune to PushRosNamespace),
    # so its namespace is injected explicitly via --ros-args -r __ns:=.
    # It is launched OUTSIDE the GroupAction, directly in the
    # LaunchDescription return list.

    namespaced_actions = [
        PushRosNamespace(robot_name),

        LogInfo(msg=f'=== AUSRA HARDWARE FULL STACK [{robot_name}] STARTING ==='),

        # Stage 0: Core (Immediate)
        # send_ns,
        robot_state_publisher,
        omni_driver,
        lidar_driver,
        mpu6050_driver,
        map_offset_node,

        # Stage 1: IMU Filter + EKF + SLAM (5 s delay)
        # Each TimerAction re-pushes the namespace for its children.
        TimerAction(
            period=5.0,
            actions=[
                GroupAction(actions=[
                    PushRosNamespace(robot_name),
                    LogInfo(msg=f'>>> [{robot_name}] Stage 1: Starting IMU Filter, EKF and SLAM...'),
                    # imu_filter_node,
                    ekf_node,
                    slam_toolbox,
                ])
            ]
        ),

        # Stage 2: Nav2 (15 s delay)
        TimerAction(
            period=15.0,
            actions=[
                GroupAction(actions=[
                    PushRosNamespace(robot_name),
                    LogInfo(msg=f'>>> [{robot_name}] Stage 2: Starting Nav2 Navigation...'),
                ])
            ] + nav2_nodes
        ),

        # Stage 3: Exploration (30 s delay)
        TimerAction(
            period=30.0,
            actions=[
                GroupAction(actions=[
                    PushRosNamespace(robot_name),
                    LogInfo(msg=f'>>> [{robot_name}] Stage 3: Starting Frontier Exploration...'),
                    exploration_server,
                ])
            ]
        ),
    ]

    # Stage 4: Nudge (Optional — uses fully-qualified topic since
    # ExecuteProcess is NOT affected by PushRosNamespace)
    if nudge_robot.perform(context) == 'true':
        namespaced_actions.append(
            TimerAction(
                period=35.0,
                actions=[
                    LogInfo(msg=f'>>> [{robot_name}] Stage 4: Nudging robot to seed SLAM...'),
                    ExecuteProcess(
                        cmd=['ros2', 'topic', 'pub', '--once',
                             f'/{robot_name}/cmd_vel', 'geometry_msgs/msg/Twist',
                             '"{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.3}}"'],
                        output='screen'
                    ),
                    TimerAction(
                        period=2.0,
                        actions=[
                            ExecuteProcess(
                                cmd=['ros2', 'topic', 'pub', '--once',
                                     f'/{robot_name}/cmd_vel', 'geometry_msgs/msg/Twist',
                                     '"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"'],
                                output='screen'
                            )
                        ]
                    )
                ]
            )
        )
    else:
        namespaced_actions.append(LogInfo(msg='>>> Skipping Nudge...'))

    # Return: micro_ros_agent runs OUTSIDE the GroupAction (it uses
    # ExecuteProcess with explicit namespace), all ROS nodes run inside.
    return [GroupAction(actions=namespaced_actions)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='ausrabot',
            description='Unique robot identifier — sets namespace AND TF frame prefix'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock'),
        DeclareLaunchArgument(
            'nudge_robot',
            default_value='false',
            description='Automatically nudge the robot to seed SLAM'),
        DeclareLaunchArgument('x', default_value='0.0', description='X position'),
        DeclareLaunchArgument('y', default_value='0.0', description='Y position'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Yaw angle'),

        OpaqueFunction(function=generate_full_hardware_stack)
    ])
