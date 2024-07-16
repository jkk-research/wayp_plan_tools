from launch import LaunchDescription
from launch_ros.actions import Node
 

def generate_launch_description():
    
    return LaunchDescription([
        Node(
            package='wayp_plan_tools',
            executable='obstacle_avoidance_trapezoid',
            name='obstacle_avoidance_trapezoid',
            output='screen',
            parameters=[
                {"detour_length_": 10.0},
                {"avoid_detour_length": 2.0},
                {"return_length_": 10.0},
                {"avoid_return_length": 2.0},
                {"offset_distance_": 2.0},
                {"avoidance_direction": "left"},
                {"lookahead_distance_": 10.0},
            ],
        ),
    ])
