from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='wayp_plan_tools',
            executable='waypoint_to_target',
            name='wayp_to_target',
            output='screen',
            parameters=[
                {"lookahead_min": 8.5},
                {"lookahead_max": 12.0},
                {"mps_alpha": 3.5},
                {"mps_beta": 5.5}, 
                {"waypoint_topic": "waypointarray"},
                {"tf_frame_id": "base_link"},
                {"tf_child_frame_id": "map"},                
            ],
        ),
    ])
