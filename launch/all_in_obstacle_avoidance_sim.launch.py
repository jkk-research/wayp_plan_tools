from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_name = 'sim_wayp_plan_tools'
    pkg_dir = get_package_share_directory(pkg_name)

    return LaunchDescription([
        # IncludeLaunchDescription(
        #         PythonLaunchDescriptionSource([
        #             FindPackageShare("gammasim_bringup"), '/launch', '/gamma.launch.py'])), 

        # start ground filter
         Node(
            package='arj_simple_perception',
            executable='lidar_filter_simple_param',
            parameters=[
                {'cloud_topic':'/gamma/points'},
                {'cloud_frame':'gamma/ouster_link/ouster'},   
                {'minZ':-1.5}, 
                {'minX':0.5},
                ],
        ), 

        #start localization
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
             arguments=[
                '--x',  '0.0',
                '--y',  '0.0',
                '--z',  '0.0',
                '--qx', '0.0',
                '--qy', '0.0',
                '--qz', '0.0',
                '--qw', '1.0',

                '--frame-id',      'map',
                '--child-frame-id','map_gamma'
            ],
        ),

        Node(
            package='lexus_bringup',
            executable='current_pose_from_tf',
            parameters=[
                {'output_topic':'/gamma/current_pose'},
                {'frame_id':'map'},
                {'child_frame_id':'base_link'},
            ],
        ),

        #start clustering
       

         Node(
            package='lidar_cluster',
            executable='euclidean_grid',
            output='screen',
            parameters=[
                {'points_in_topic': 'lidar_filter_output'},
                {'points_out_topic': 'clustered_points'},
                {'marker_out_topic': 'clustered_marker'},
                {'tolerance': 5.0},
                {'max_cluster_size': 4000},
                {'voxel_leaf_size': 3.0},
                {'min_points_number_per_voxel': 5},
                {'verbose1': False},
                {'verbose2': False},
            ]
        ),

        # load waypoints

         Node(
            package='wayp_plan_tools',
            executable='waypoint_loader',
            output='screen',
            namespace='sim1', 
            parameters=[
                {"file_name":"sim_waypoints3.csv"},
                {"file_dir": pkg_dir +"/csv"},
                
            ]
        ),
    ])