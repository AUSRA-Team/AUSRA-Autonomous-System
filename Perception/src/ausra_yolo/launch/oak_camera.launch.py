from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='ausra_1',
        description='Robot namespace — ausra_1, ausra_2, or ausra_3'
    )

    robot_name = LaunchConfiguration('robot_name')

    config_file = PathJoinSubstitution([
        FindPackageShare('ausra_yolo'),
        'config',
        'oak_d_lite.yaml'
    ])

    oak_group = GroupAction([
        PushRosNamespace(robot_name),

        Node(
            package='depthai_ros_driver',
            executable='camera_node',
            output='screen',
            parameters=[
                config_file,
            ],
        ),
    ])

    return LaunchDescription([
        robot_name_arg,
        oak_group,
    ])