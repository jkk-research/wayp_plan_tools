from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='wayp_plan_tools',
            executable='single_goal_pursuit',
            name='pure_pursuit',
            output='screen',
            parameters=[
                {"cmd_topic": "cmd_vel"},
                {"wheelbase": 2.789},
                {"waypoint_topic": "targetpoints"},
            ],
        )
    ])
