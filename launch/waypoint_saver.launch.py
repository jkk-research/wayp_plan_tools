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
            executable='waypoint_saver',
            name='wayp_saver',
            output='screen',
            parameters=[
                {"file_dir": pkg_dir + "/csv"},
                #{"file_dir": "/mnt/c/waypoints"},
                {"file_name": "saved_waypoints1.csv"},
                #{"topic_based_saving": True}, # False default (which is tf based)
                #{"pose_topic": "/odom"},
                #{"pose_topic_type": "Odometry"},
                #{"tf_frame_id", "map"},
                #{"tf_child_frame_id", "base_link"},
            ],
        )
    ])