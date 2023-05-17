from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    pkg_name = 'wayp_plan_tools'
    pkg_dir = os.popen('/bin/bash -c "cd && source /usr/share/colcon_cd/function/colcon_cd.sh && colcon_cd %s && pwd"' % pkg_name).read().strip()
    #print(pkg_dir)

    return LaunchDescription([
        Node(
            package='wayp_plan_tools',
            executable='waypoint_to_target',
            name='wayp_to_target',
            output='screen',
            parameters=[
                {"lookahead_distance": 3.0}
                ],
        ),

        Node(
            package='wayp_plan_tools',
            executable='waypoint_loader',
            name='wayp_load',
            output='screen',
            parameters=[
                #{"file_dir": "/mnt/c/waypoints"},
                {"file_dir": pkg_dir + "/csv"},
                {"file_name": "example01.csv"}],
        ),


        Node(
            package='wayp_plan_tools',
            executable='single_goal_pursuit',
            name='pure_pursuit',
            output='screen',
            parameters=[
                {"cmd_topic": "lexus3/cmd_vel"},
                {"wheelbase": 2.789}
                ],
        )
    ])
