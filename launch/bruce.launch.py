# ROS 2 launch file for the BRUCE / Aracati-style datasets.
#
# Ported from the ROS 1 launch/bruce.launch.  Pieces of the original file that do
# not exist in this repository and were therefore dropped:
#   * executable "direct_sonar_odometry_node2" - no such CMake target exists.  The
#     real target that consumes config_bruce.yaml is "direct_sonar_odometry_node",
#     which is what this file launches.
#   * node "bruce_save.py" - that script is not part of this repository.
#
# Other differences with respect to the original XML file:
#   * the ROS 1 "tf/static_transform_publisher" node became a
#     "tf2_ros/static_transform_publisher" node using the named-argument form;
#   * optional rosbag2 playback was added (the "bag" / "bag_args" arguments);
#   * an additional identity map -> odom static transform is published (the
#     original only published odom -> orb_slam).  DISO publishes its markers,
#     poses and paths in "map" while the point cloud and the odom -> base_link
#     transform live in "odom"; without that link RViz, whose fixed frame is
#     "map", cannot connect the two halves of the TF tree.

import shlex

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _bag_play(context, *args, **kwargs):
    """Run "ros2 bag play <bag> <bag_args>" when the "bag" argument is not empty."""
    bag = LaunchConfiguration('bag').perform(context)
    if not bag:
        return []
    extra_args = shlex.split(LaunchConfiguration('bag_args').perform(context))
    return [
        ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bag] + extra_args,
            output='screen',
        )
    ]


def generate_launch_description():
    config = LaunchConfiguration('config')
    rviz = LaunchConfiguration('rviz')
    output_dir = LaunchConfiguration('output_dir')
    debug_dir = LaunchConfiguration('debug_dir')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_config = DeclareLaunchArgument(
        'config',
        default_value=PathJoinSubstitution([
            FindPackageShare('direct_sonar_odometry'),
            'config',
            'config_bruce.yaml',
        ]),
        description='Path to the DISO settings file.',
    )
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Start RViz2 with the DISO configuration.',
    )
    declare_output_dir = DeclareLaunchArgument(
        'output_dir',
        default_value='',
        description='Directory for the trajectory dumps.  Empty disables them.',
    )
    declare_debug_dir = DeclareLaunchArgument(
        'debug_dir',
        default_value='',
        description='Directory for the per-edge chi2 dumps.  Empty disables them.',
    )
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use the /clock topic instead of the system clock.',
    )
    declare_bag = DeclareLaunchArgument(
        'bag',
        default_value='',
        description='Path to a rosbag2 directory to play.  Empty does not play anything.',
    )
    declare_bag_args = DeclareLaunchArgument(
        'bag_args',
        default_value='--clock -r 0.8',
        description='Extra arguments passed to "ros2 bag play".',
    )

    static_tf_map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_odom',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    static_tf_odom_to_orb_slam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_to_orb_slam',
        arguments=['--x', '0', '--y', '0', '--z', '0',
                   '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
                   '--frame-id', 'odom', '--child-frame-id', 'orb_slam'],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    direct_sonar_odometry_node = Node(
        package='direct_sonar_odometry',
        executable='direct_sonar_odometry_node',
        name='direct_sonar_odometry_node',
        output='screen',
        arguments=[config],
        parameters=[{
            'settings_file': config,
            'output_dir': output_dir,
            'debug_dir': debug_dir,
            'use_sim_time': use_sim_time,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz_direct_sonar',
        condition=IfCondition(rviz),
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('direct_sonar_odometry'),
            'launch',
            'sonar_odometry.rviz',
        ])],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        declare_config,
        declare_rviz,
        declare_output_dir,
        declare_debug_dir,
        declare_use_sim_time,
        declare_bag,
        declare_bag_args,
        static_tf_map_to_odom,
        static_tf_odom_to_orb_slam,
        direct_sonar_odometry_node,
        rviz_node,
        OpaqueFunction(function=_bag_play),
    ])
