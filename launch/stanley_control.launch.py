from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='wayp_plan_tools',
            executable='stanley_control',
            name='stanley_control',
            output='screen',
            parameters=[
                {"cmd_topic": "cmd_vel"},
                {"waypoint_topic": "waypointarray"},  # global path (map frame), not the targetpoints
                {"map_frame": "map"},
                {"base_frame": "base_link"},  # center of the rear axle
                {"wheelbase": 2.789},
                {"cross_track_err_gain": 1.0},  # k in Eq. 7
                {"heading_err_gain": 1.0},
                {"k_soft": 1.0},
                {"max_steer": 0.6},
            ],
        )
    ])
