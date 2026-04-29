import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _as_float(name, value):
    try:
        return float(value)
    except ValueError as exc:
        raise RuntimeError(f"Launch argument '{name}' must be a number, got '{value}'") from exc


def _launch_setup(context, *args, **kwargs):
    go2_core_dir = get_package_share_directory('go2_core')
    go2_perception_dir = get_package_share_directory('go2_perception')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    params_file = LaunchConfiguration('params_file').perform(context)
    slam_params_file = LaunchConfiguration('slam_params_file').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    autostart = LaunchConfiguration('autostart').perform(context)

    slam_delay = _as_float('slam_delay', LaunchConfiguration('slam_delay').perform(context))
    navigation_delay = _as_float('navigation_delay', LaunchConfiguration('navigation_delay').perform(context))
    rviz_bridge_delay = _as_float(
        'rviz_bridge_delay',
        LaunchConfiguration('rviz_bridge_delay').perform(context),
    )

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_core_dir, 'launch', 'go2_base.launch.py')
            ),
            launch_arguments={
                'video_enable': LaunchConfiguration('video_enable').perform(context),
                'tcp_enable': LaunchConfiguration('tcp_enable').perform(context),
                'tcp_host': LaunchConfiguration('tcp_host').perform(context),
                'tcp_port': LaunchConfiguration('tcp_port').perform(context),
                'target_fps': LaunchConfiguration('target_fps').perform(context),
                'image_topic': LaunchConfiguration('image_topic').perform(context),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(go2_perception_dir, 'launch', 'go2_pointcloud_process.launch.py')
            )
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_footprint',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        ),
        TimerAction(
            period=slam_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(slam_toolbox_dir, 'launch', 'online_async_launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'params_file': slam_params_file,
                    }.items(),
                )
            ],
        ),
        TimerAction(
            period=navigation_delay,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                    ),
                    launch_arguments={
                        'use_sim_time': use_sim_time,
                        'autostart': autostart,
                        'params_file': params_file,
                        'map_subscribe_transient_local': 'true',
                    }.items(),
                )
            ],
        ),
    ]

    if _as_bool(LaunchConfiguration('enable_rviz_bridge').perform(context)):
        actions.append(
            TimerAction(
                period=rviz_bridge_delay,
                actions=[
                    Node(
                        package='go2_navigation',
                        executable='rviz_tcp_bridge.py',
                        name='go2_rviz_tcp_bridge',
                        output='screen',
                        arguments=[
                            '--send-host',
                            LaunchConfiguration('rviz_bridge_send_host').perform(context),
                            '--send-port',
                            LaunchConfiguration('rviz_bridge_send_port').perform(context),
                            '--recv-host',
                            LaunchConfiguration('rviz_bridge_recv_host').perform(context),
                            '--recv-port',
                            LaunchConfiguration('rviz_bridge_recv_port').perform(context),
                            '--goal-mode',
                            LaunchConfiguration('rviz_bridge_goal_mode').perform(context),
                            '--image-max-hz',
                            LaunchConfiguration('rviz_bridge_image_max_hz').perform(context),
                            '--pointcloud-max-hz',
                            LaunchConfiguration('rviz_bridge_pointcloud_max_hz').perform(context),
                        ],
                    )
                ],
            )
        )

    return actions


def generate_launch_description():
    go2_navigation_dir = get_package_share_directory('go2_navigation')
    default_params_file = os.path.join(go2_navigation_dir, 'config', 'nav2_params.yaml')
    default_slam_params_file = os.path.join(go2_navigation_dir, 'config', 'slam_toolbox_live.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        DeclareLaunchArgument('slam_params_file', default_value=default_slam_params_file),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('cyclonedds_interface', default_value='eth0'),
        DeclareLaunchArgument('video_enable', default_value='false'),
        DeclareLaunchArgument('tcp_enable', default_value='false'),
        DeclareLaunchArgument('tcp_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('tcp_port', default_value='5432'),
        DeclareLaunchArgument('target_fps', default_value='30'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('slam_delay', default_value='2.0'),
        DeclareLaunchArgument('navigation_delay', default_value='8.0'),
        DeclareLaunchArgument('enable_rviz_bridge', default_value='false'),
        DeclareLaunchArgument('rviz_bridge_delay', default_value='8.0'),
        DeclareLaunchArgument('rviz_bridge_send_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('rviz_bridge_send_port', default_value='16000'),
        DeclareLaunchArgument('rviz_bridge_recv_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('rviz_bridge_recv_port', default_value='16001'),
        DeclareLaunchArgument('rviz_bridge_goal_mode', default_value='action'),
        DeclareLaunchArgument('rviz_bridge_image_max_hz', default_value='5.0'),
        DeclareLaunchArgument('rviz_bridge_pointcloud_max_hz', default_value='2.0'),
        UnsetEnvironmentVariable(name='ROS_DOMAIN_ID'),
        SetEnvironmentVariable(name='ROS_LOCALHOST_ONLY', value='0'),
        SetEnvironmentVariable(name='RMW_IMPLEMENTATION', value='rmw_cyclonedds_cpp'),
        SetEnvironmentVariable(
            name='CYCLONEDDS_URI',
            value=[
                '<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="',
                LaunchConfiguration('cyclonedds_interface'),
                '"/></Interfaces></General></Domain></CycloneDDS>',
            ],
        ),
        OpaqueFunction(function=_launch_setup),
    ])
