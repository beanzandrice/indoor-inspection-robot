import math
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
    TimerAction,
    UnsetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return value.strip().lower() in ('1', 'true', 'yes', 'on')


def _as_float(name, value):
    try:
        parsed = float(value)
    except ValueError as exc:
        raise RuntimeError(f"Launch argument '{name}' must be a number, got '{value}'") from exc
    if not math.isfinite(parsed) or parsed < 0.0:
        raise RuntimeError(f"Launch argument '{name}' must be finite and non-negative")
    return parsed


def _pose_message(context):
    x = LaunchConfiguration('initial_x').perform(context)
    y = LaunchConfiguration('initial_y').perform(context)
    z = LaunchConfiguration('initial_z').perform(context)
    qx = LaunchConfiguration('initial_qx').perform(context)
    qy = LaunchConfiguration('initial_qy').perform(context)
    qz = LaunchConfiguration('initial_qz').perform(context)
    qw = LaunchConfiguration('initial_qw').perform(context)

    return (
        "{"
        "header: {frame_id: 'map'}, "
        "pose: {"
        "pose: {"
        f"position: {{x: {x}, y: {y}, z: {z}}}, "
        f"orientation: {{x: {qx}, y: {qy}, z: {qz}, w: {qw}}}"
        "}, "
        "covariance: ["
        "0.25, 0.0, 0.0, 0.0, 0.0, 0.0, "
        "0.0, 0.25, 0.0, 0.0, 0.0, 0.0, "
        "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
        "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
        "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, "
        "0.0, 0.0, 0.0, 0.0, 0.0, 0.068"
        "]}"
        "}"
    )


def _initial_pose_action(context):
    return ExecuteProcess(
        cmd=[
            'ros2',
            'topic',
            'pub',
            '--once',
            '/initialpose',
            'geometry_msgs/msg/PoseWithCovarianceStamped',
            _pose_message(context),
        ],
        output='screen',
    )


def _readiness_gate(label, timeout, dependent_actions, **condition):
    """Start actions when a ROS graph condition is ready or its fallback expires."""
    condition_arguments = []
    for name, value in condition.items():
        if value:
            condition_arguments.extend([f'--{name.replace("_", "-")}', value])

    probe = Node(
        package='go2_navigation',
        executable='wait_for_ros.py',
        name=f'wait_for_{label}',
        output='screen',
        arguments=[
            *condition_arguments,
            '--timeout',
            str(timeout),
            '--label',
            label,
        ],
    )

    started_at = time.monotonic()

    def start_or_preserve_fallback(event, _context):
        if event.returncode == 0:
            return list(dependent_actions)
        remaining = max(timeout - (time.monotonic() - started_at), 0.0)
        if remaining <= 0.0:
            return list(dependent_actions)
        return [TimerAction(period=remaining, actions=list(dependent_actions))]

    return [
        RegisterEventHandler(
            OnProcessExit(
                target_action=probe,
                on_exit=start_or_preserve_fallback,
            )
        ),
        probe,
    ]


def _launch_setup(context, *args, **kwargs):
    go2_core_dir = get_package_share_directory('go2_core')
    go2_perception_dir = get_package_share_directory('go2_perception')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    params_file = LaunchConfiguration('params_file').perform(context)
    map_file = LaunchConfiguration('map').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    autostart = LaunchConfiguration('autostart').perform(context)

    localization_delay = _as_float(
        'localization_delay',
        LaunchConfiguration('localization_delay').perform(context),
    )
    navigation_delay = _as_float(
        'navigation_delay',
        LaunchConfiguration('navigation_delay').perform(context),
    )
    initial_pose_delay = _as_float(
        'initial_pose_delay',
        LaunchConfiguration('initial_pose_delay').perform(context),
    )
    initial_pose_retry_delay = _as_float(
        'initial_pose_retry_delay',
        LaunchConfiguration('initial_pose_retry_delay').perform(context),
    )
    map_server_reset_delay = _as_float(
        'map_server_reset_delay',
        LaunchConfiguration('map_server_reset_delay').perform(context),
    )
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
                # The copied project supplies its own camera node below. Keep the
                # stock go2_core video path disabled because some installs do not
                # include go2_core/video_stream_node.py.
                'video_enable': 'false',
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
    ]

    if _as_bool(LaunchConfiguration('enable_go2_camera').perform(context)):
        actions.append(
            Node(
                package='go2_navigation',
                executable='go2_camera_image_publisher.py',
                name='go2_camera_image_publisher',
                output='screen',
                additional_env={
                    'LD_PRELOAD': LaunchConfiguration('camera_ld_preload').perform(context),
                },
                parameters=[{
                    'image_topic': LaunchConfiguration('image_topic').perform(context),
                    'camera_frame_id': LaunchConfiguration('camera_frame_id').perform(context),
                    'target_fps': _as_float(
                        'target_fps',
                        LaunchConfiguration('target_fps').perform(context),
                    ),
                    'gstreamer_pipeline': LaunchConfiguration('camera_gstreamer_pipeline').perform(context),
                }],
            )
        )

    if _as_bool(LaunchConfiguration('enable_joint_tf').perform(context)):
        actions.append(
            Node(
                package='go2_navigation',
                executable='go2_joint_tf_publisher.py',
                name='go2_joint_tf_publisher',
                output='screen',
                parameters=[{
                    'lowstate_topics': LaunchConfiguration('lowstate_topics').perform(context),
                    'publish_hz': _as_float(
                        'joint_tf_publish_hz',
                        LaunchConfiguration('joint_tf_publish_hz').perform(context),
                    ),
                }],
            )
        )

    actions.append(
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_base_footprint',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        )
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
        }.items(),
    )
    actions.extend(
        _readiness_gate(
            'localization_input',
            localization_delay,
            [localization],
            topic='/scan',
        )
    )

    navigation = IncludeLaunchDescription(
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
    actions.extend(
        _readiness_gate(
            'navigation_map_server',
            navigation_delay,
            [navigation],
            lifecycle_node='/map_server',
        )
    )

    if _as_bool(LaunchConfiguration('enable_rviz_bridge').perform(context)):
        bridge_arguments = [
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
            '--protocol',
            LaunchConfiguration('rviz_bridge_protocol').perform(context),
            '--topic-allowlist',
            LaunchConfiguration('rviz_bridge_topic_allowlist').perform(context),
            '--queue-capacity',
            LaunchConfiguration('rviz_bridge_queue_capacity').perform(context),
            '--max-frame-bytes',
            LaunchConfiguration('rviz_bridge_max_frame_bytes').perform(context),
            '--max-buffer-bytes',
            LaunchConfiguration('rviz_bridge_max_buffer_bytes').perform(context),
            '--write-timeout',
            LaunchConfiguration('rviz_bridge_write_timeout').perform(context),
            '--tf-max-hz',
            LaunchConfiguration('rviz_bridge_tf_max_hz').perform(context),
            '--image-max-hz',
            LaunchConfiguration('rviz_bridge_image_max_hz').perform(context),
            '--pointcloud-max-hz',
            LaunchConfiguration('rviz_bridge_pointcloud_max_hz').perform(context),
        ]
        topic_rate_limits = LaunchConfiguration('rviz_bridge_topic_rate_limits').perform(context)
        if topic_rate_limits:
            bridge_arguments.extend(['--topic-rate-limits', topic_rate_limits])
        if _as_bool(LaunchConfiguration('rviz_bridge_debug_all_topics').perform(context)):
            bridge_arguments.append('--debug-all-topics')

        bridge = Node(
            package='go2_navigation',
            executable='rviz_tcp_bridge.py',
            name='go2_rviz_tcp_bridge',
            output='screen',
            arguments=bridge_arguments,
        )
        actions.extend(
            _readiness_gate(
                'rviz_bridge_tf',
                rviz_bridge_delay,
                [bridge],
                topic='/tf',
            )
        )

    if _as_bool(LaunchConfiguration('publish_initial_pose').perform(context)):
        actions.extend(
            _readiness_gate(
                'initial_pose_amcl',
                initial_pose_delay,
                [_initial_pose_action(context)],
                lifecycle_node='/amcl',
            )
        )
        if initial_pose_retry_delay > 0.0:
            actions.append(
                TimerAction(
                    period=initial_pose_delay + initial_pose_retry_delay,
                    actions=[_initial_pose_action(context)],
                )
            )

    if _as_bool(LaunchConfiguration('reset_map_server').perform(context)):
        actions.append(
            TimerAction(
                period=map_server_reset_delay,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'lifecycle', 'set', '/map_server', 'deactivate'],
                        output='screen',
                    ),
                    TimerAction(
                        period=2.0,
                        actions=[
                            ExecuteProcess(
                                cmd=['ros2', 'lifecycle', 'set', '/map_server', 'activate'],
                                output='screen',
                            )
                        ],
                    ),
                ],
            )
        )

    return actions


def generate_launch_description():
    go2_navigation_dir = get_package_share_directory('go2_navigation')
    default_params_file = os.path.join(go2_navigation_dir, 'config', 'nav2_params.yaml')
    default_map_file = os.path.join(go2_navigation_dir, 'maps', 'room_map_toolbox.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map_file),
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('cyclonedds_interface', default_value='eth0'),
        DeclareLaunchArgument('video_enable', default_value='false'),
        DeclareLaunchArgument('tcp_enable', default_value='false'),
        DeclareLaunchArgument('tcp_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('tcp_port', default_value='5432'),
        DeclareLaunchArgument('target_fps', default_value='30'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('enable_go2_camera', default_value='false'),
        DeclareLaunchArgument('camera_frame_id', default_value='go2_front_camera'),
        DeclareLaunchArgument('camera_ld_preload', default_value='/lib/aarch64-linux-gnu/libgomp.so.1'),
        DeclareLaunchArgument(
            'camera_gstreamer_pipeline',
            default_value=(
                'udpsrc address=230.1.1.1 port=1720 multicast-iface=eth0 ! '
                'application/x-rtp, media=video, encoding-name=H264 ! '
                'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! '
                'video/x-raw,width=1280,height=720,format=BGR ! '
                'appsink name=go2_camera_sink drop=true sync=false max-buffers=1'
            ),
        ),
        DeclareLaunchArgument('enable_joint_tf', default_value='true'),
        DeclareLaunchArgument('lowstate_topics', default_value='lowstate,lf/lowstate'),
        DeclareLaunchArgument('joint_tf_publish_hz', default_value='20.0'),
        DeclareLaunchArgument('localization_delay', default_value='2.0'),
        DeclareLaunchArgument('navigation_delay', default_value='6.0'),
        DeclareLaunchArgument('publish_initial_pose', default_value='true'),
        DeclareLaunchArgument('initial_pose_delay', default_value='16.0'),
        DeclareLaunchArgument('initial_pose_retry_delay', default_value='6.0'),
        DeclareLaunchArgument('initial_x', default_value='0.5153582096'),
        DeclareLaunchArgument('initial_y', default_value='-2.3085870743'),
        DeclareLaunchArgument('initial_z', default_value='0.0'),
        DeclareLaunchArgument('initial_qx', default_value='0.0'),
        DeclareLaunchArgument('initial_qy', default_value='0.0'),
        DeclareLaunchArgument('initial_qz', default_value='0.0'),
        DeclareLaunchArgument('initial_qw', default_value='1.0'),
        DeclareLaunchArgument('reset_map_server', default_value='true'),
        DeclareLaunchArgument('map_server_reset_delay', default_value='24.0'),
        DeclareLaunchArgument('enable_rviz_bridge', default_value='false'),
        DeclareLaunchArgument('rviz_bridge_delay', default_value='8.0'),
        DeclareLaunchArgument('rviz_bridge_send_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('rviz_bridge_send_port', default_value='16000'),
        DeclareLaunchArgument('rviz_bridge_recv_host', default_value='127.0.0.1'),
        DeclareLaunchArgument('rviz_bridge_recv_port', default_value='16001'),
        DeclareLaunchArgument('rviz_bridge_goal_mode', default_value='action'),
        DeclareLaunchArgument('rviz_bridge_protocol', default_value='binary'),
        DeclareLaunchArgument('rviz_bridge_topic_allowlist', default_value='default'),
        DeclareLaunchArgument('rviz_bridge_topic_rate_limits', default_value=''),
        DeclareLaunchArgument('rviz_bridge_debug_all_topics', default_value='false'),
        DeclareLaunchArgument('rviz_bridge_queue_capacity', default_value='32'),
        DeclareLaunchArgument('rviz_bridge_max_frame_bytes', default_value='16777216'),
        DeclareLaunchArgument('rviz_bridge_max_buffer_bytes', default_value='20971520'),
        DeclareLaunchArgument('rviz_bridge_write_timeout', default_value='2.0'),
        DeclareLaunchArgument('rviz_bridge_tf_max_hz', default_value='20.0'),
        DeclareLaunchArgument('rviz_bridge_image_max_hz', default_value='2.0'),
        DeclareLaunchArgument('rviz_bridge_pointcloud_max_hz', default_value='0.5'),
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
