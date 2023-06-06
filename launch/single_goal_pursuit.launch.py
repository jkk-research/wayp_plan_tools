from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    #pkg_name = 'wayp_plan_tools'
    #pkg_dir = os.popen('/bin/bash -c "cd && source /usr/share/colcon_cd/function/colcon_cd.sh && colcon_cd %s && pwd"' % pkg_name).read().strip()
    #print(pkg_dir)

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
