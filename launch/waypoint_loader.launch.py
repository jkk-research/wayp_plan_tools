from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    pkg_name = 'wayp_plan_tools'
    pkg_dir = os.popen('/bin/bash -c "source /usr/share/colcon_cd/function/colcon_cd.sh && colcon_cd %s && pwd"' % pkg_name).read().strip()
    #print(pkg_dir)

    return LaunchDescription([
        Node(
            package='wayp_plan_tools',
            executable='waypoint_loader',
            name='wayp_load',
            output='screen',
            parameters=[
                #{"file_dir": "/mnt/c/waypoints"},
                # {"file_dir": pkg_dir + "/csv"},
                # {"file_name": "example01.csv"}],
                {"file_dir": "/mnt/bag/waypoints/"},
                {"file_name": "gyor1.csv"}],
        )
    ])