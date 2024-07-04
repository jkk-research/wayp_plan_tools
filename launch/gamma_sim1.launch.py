from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    pkg_name_w = 'wayp_plan_tools'
    pkg_name_s = 'sim_wayp_plan_tools'
    pkg_dir_w = get_package_share_directory(pkg_name_w)
    pkg_dir_s = get_package_share_directory(pkg_name_s)
    #print(pkg_dir)

    return LaunchDescription(
        [
        Node(
            package='wayp_plan_tools',
            executable='waypoint_loader_with_stops',
            namespace='sim1',
            output='screen',
            parameters=[
                {"file_dir": pkg_dir_s + "/csv"},
                #{"file_dir": "/mnt/bag/waypoints/"},
                {"file_name": "sim_waypoints3.csv"},
                {"per_waypoint_display": 5}, # display speed every 5th waypoint 
                {"stop_interval": 22.0}, # stop every X meters
                {"stop_decceleration": 0.2}, #  m/s^2
                {"start_acceleration": 0.2}, #  m/s^2
                ],
            ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("sim_wayp_plan_tools"), '/launch/', 'gazebo_bridge.launch.py'])
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("sim_wayp_plan_tools"), '/launch/', 'rviz1.launch.py'])
        ),

        TimerAction(
            period=4.0, # delay / wait in seconds
            actions=[
                Node(
                    package='wayp_plan_tools',
                    executable='waypoint_to_target',
                    output='screen',
                    namespace='sim1',
                    parameters=[
                        {"lookahead_min": 2.5},
                        {"lookahead_max": 4.5},
                        {"mps_alpha": 1.5},
                        {"mps_beta": 4.5}, 
                        {"waypoint_topic": "waypointarray"},
                        {"tf_frame_id": "base_link"},
                        {"tf_child_frame_id": "map"},
                        {"interpolate_waypoints": True},
                        {"stop_duration": 8.0}, # stop for X seconds
                        {"creep_duration": 6.0}, # creep for X seconds
                    ],
                ),
                Node(
                    package='sim_wayp_plan_tools',
                    executable='visuals',
                    output='screen',
                    namespace='sim1',
                    parameters=[
                        {"marker_topic": "marker_steering"},
                        {"mod_limit": 100}, # modulo limit for path size (publish every n-th message)
                        {"path_size": 3000},
                        {"pose_frame": "base_link"}, 
                        {"publish_steer_marker": True},
                    ],
                ),
            ]
        ),    
        TimerAction(
            period=5.0, # delay / wait in seconds
            actions=[
                Node(
                    package='wayp_plan_tools',
                    executable='single_goal_pursuit',
                    namespace='sim1',
                    output='screen',
                    parameters=[
                            {"cmd_topic": "/model/vehicle_blue/cmd_vel"},
                            {"wheelbase": 1.0}, # from the /usr/share/ignition/ignition-gazebo6/worlds/ackermann_steering.sdf file wheel_base parameter
                            {"waypoint_topic": "targetpoints"},
                        ],
                ),
                # IncludeLaunchDescription(
                #     PythonLaunchDescriptionSource([
                #         FindPackageShare("sim_wayp_plan_tools"), '/launch/', 'single_goal_pursuit.launch.py'])
                # ),
            ]
        ),         
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            name='fx',
            output='screen', ##  TODO: supress output
            arguments=['__log_level:=warn'], ## TODO: supress output
            additional_env={'PYTHONUNBUFFERED': '1', 'ROS_PYTHON_LOG_CONFIG_FILE': os.devnull}, ## TODO: supress output
            ),
        ]
    )
