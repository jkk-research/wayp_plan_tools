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
                {"lookahead_min": 8.5},
                {"lookahead_max": 12.0},
                {"mps_alpha": 3.5},
                {"mps_beta": 5.5}, 
                {"waypoint_topic": "waypointarray"},
                {"tf_frame_id": "base_link"},
                {"tf_child_frame_id": "map"},                
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
                {"file_name": "example01.csv"}
                {"per_waypoint_display": 10}, # display speed every 10th waypoint 
            ],
        ),


        Node(
            package='wayp_plan_tools',
            executable='single_goal_pursuit',
            name='pure_pursuit',
            output='screen',
            parameters=[
                {"cmd_topic": "cmd_vel"},
                {"wheelbase": 2.789},
            ],
        )
    ])
