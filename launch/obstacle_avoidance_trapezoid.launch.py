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
                # {"sample_param": 4.20},             
            ],
        ),
    ])
