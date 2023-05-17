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
            executable='waypoint_to_target',
            name='wayp_to_target',
            output='screen',
            parameters=[
                {"lookahead_min": 8.5},
                {"lookahead_max": 12.0},
                {"mps_alpha": 3.5},
                {"mps_beta": 5.5}, 
                {"waypoint_topic": "lexus3/waypointarray"}
                ],
        ),
    ])
