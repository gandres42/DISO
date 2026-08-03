# ROS 2 launch file for the simulated (UUV / rexrov) sequences.
#
# Ported from the ROS 1 launch/sim.launch.  Pieces of the original file that do
# not exist in this repository:
#   * node "sim_node" - it would have been built from src/run_sim.cpp, which is
#     not in the repository (the CMake target is commented out upstream).  It is
#     replaced here by "direct_sonar_odometry_node", the generic entry point:
#     both are just System(settings_file) + runRos().
#   * <include file="$(find robot_localization)/launch/ekf_rexrov.launch"/> -
#     robot_localization is not a dependency of this workspace and that launch
#     file does not ship with it.
#   * node "bruce_save.py" - that script is not part of this repository.
#
# CAVEAT, inherited from upstream: config/config_sim.yaml names
# "/odometry/filtered" as OdomTopic, which is nav_msgs/Odometry, but
# System::runRos() only wires up the geometry_msgs/PoseStamped variant
# (System::frameLoad2, the Odometry one, is commented out upstream).  A simulated
# run therefore needs either an Odometry -> PoseStamped republisher or that block
# re-enabled.  That topic also came from the robot_localization EKF dropped above.
#
# What is kept from the original: the identity map -> odom static transform and
# RViz.  Other differences:
#   * the "repub_gt" node was dropped along with the rest of the ground-truth
#     republishing, so the only topics this pipeline needs are the sonar image
#     and the odometry prior named by the settings file;
#   * the ROS 1 "tf/static_transform_publisher" node became a
#     "tf2_ros/static_transform_publisher" node using the named-argument form;
#   * optional rosbag2 playback was added (the "bag" / "bag_args" arguments).

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
            'config_sim.yaml',
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
        direct_sonar_odometry_node,
        rviz_node,
        OpaqueFunction(function=_bag_play),
    ])
