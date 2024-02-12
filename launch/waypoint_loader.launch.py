from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = 'wayp_plan_tools'
    pkg_dir = get_package_share_directory(pkg_name)
    #print(pkg_dir)

    return LaunchDescription([
        Node(
            package='wayp_plan_tools',
            executable='waypoint_loader',
            name='wayp_load',
            output='screen',
            parameters=[
                {"file_dir": pkg_dir + "/csv"},
                #{"file_dir": "/mnt/bag/waypoints/"},
                {"file_name": "example01.csv"},
                {"per_waypoint_display": 5}, # display speed every 5th waypoint 
            ],
        )
    ])