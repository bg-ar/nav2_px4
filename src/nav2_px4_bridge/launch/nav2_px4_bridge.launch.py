
# ARI-Nav2-PX4-Bridge: A simple ROS 2 node to bridge Nav2 cmd_vel to PX4 offboard control mode.
# By BG Kang (github.com/bgkng), 2024. Open source under Apache 2.0 license.

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="nav2_px4_bridge",
            executable="nav2_px4_bridge_node",
            name="nav2_px4_bridge",
            output="screen",
            parameters=[{
                "px4_namespace": "",     # 예: PX4 uxrce_dds_client -n drone 이면 "drone"
                "topic_suffix": "",      # 예: 토픽이 *_v1 이면 "_v1"
                "cmd_vel_topic": "/cmd_vel",
                "publish_rate_hz": 50.0,
                "cmd_vel_timeout_s": 0.5,
                "cmd_vel_is_body_flu": True,
                "max_vxy": 2.0,
                "max_vz": 1.0,
                "max_yawrate": 1.5,
                "stream_always": True,
                "auto_offboard_and_arm": False,
                "warmup_cycles": 50
            }]
        )
    ])