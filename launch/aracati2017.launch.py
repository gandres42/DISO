# ROS 2 launch file for the aracati2017 sequence.
#
# Ported from the ROS 1 launch/aracati2017.launch.  Differences with respect to
# the original XML file:
#   * the ROS 1 "tf/static_transform_publisher" nodes became
#     "tf2_ros/static_transform_publisher" nodes using the named-argument form;
#   * an additional identity map -> odom static transform is published.  DISO
#     publishes its markers, poses and paths in "map", while the point cloud and
#     the odom -> base_link transform live in "odom"; without that link RViz
#     cannot connect the two halves of the TF tree;
#   * the commented-out "bruce_save.py" node of the original file is dropped -
#     that script does not exist in this repository;
#   * a "cmd_vel_odom" node supplies /odom_pose, the odometry prior named by
#     config/config_aracati2017.yaml.  The bag does not contain that topic;
#     upstream it came from the companion Aracati2017_DISO package.  See
#     src/CmdVelOdom.cpp.  Set "odom_source:=external" to provide it yourself;
#   * optional rosbag2 playback was added (the "bag" / "bag_args" arguments).

import shlex

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)

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
            'config_aracati2017.yaml',
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
    declare_odom_source = DeclareLaunchArgument(
        'odom_source',
        default_value='cmd_vel',
        choices=['cmd_vel', 'external'],
        description=(
            'Where the odometry prior on "OdomTopic" comes from.  "cmd_vel" runs the '
            'bundled dead-reckoning node; "external" assumes something else publishes it.'
        ),
    )

    cmd_vel_odom_node = Node(
        package='direct_sonar_odometry',
        executable='cmd_vel_odom',
        name='cmd_vel_odom',
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", LaunchConfiguration('odom_source'), "' == 'cmd_vel'"])),
        parameters=[{'use_sim_time': use_sim_time}],
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

    repub_gt_node = Node(
        package='direct_sonar_odometry',
        executable='repub_gt',
        name='repub_gt_node',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    direct_sonar_odometry_node = Node(
        package='direct_sonar_odometry',
        executable='aracati2017_node',
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
        declare_odom_source,
        static_tf_map_to_odom,
        static_tf_odom_to_orb_slam,
        cmd_vel_odom_node,
        repub_gt_node,
        direct_sonar_odometry_node,
        rviz_node,
        OpaqueFunction(function=_bag_play),
    ])
