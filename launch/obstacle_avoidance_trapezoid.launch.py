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
                {"detour_length_": 4.0},                #kiteres hossza a trapez elso oldalan (m), 
                {"avoid_detour_length": 6.0},           #a trapez hosszu oldal elso felenek a hossza (m)
                {"avoid_return_length": 6.0},           #a trapez hosszu oldal masodik felenek a hossza (m)
                {"return_length_": 4.0},                #a visszatero szakasz hossza (m)
                {"offset_distance_": 3.0},              #a trapez szelessege (m), az eredeti uttol valo tavolsag
                {"avoidance_direction": "left"},        #elkerules iranya
                {"lookahead_distance_": 50},           #az eloretekintes hossza (m)
                {"min_distance_treshold": 4.0},         #Az akadaly és a waypont kozotti tavolsag, amely alatt az akadalyt figyelembe vesszuk (m) 
                {"sensitivity": 2.0},                   #az akadaly erzekenysege, ennyi elofordulas kell a figyelembe vetelhez minimum (db)
                {"waypoint_topic":"waypointarray"},     #a waypointokat tartalmazo topik
                {"pose_topic":"rotated_pose"},          #Az auto poziciojat tartalmazo topik
                {"obstacle_topic":"clustered_marker"},  #Az akadalyokat tartalmazo topik, marker array
                {"lidar_frame":"lexus3/os_center_a_laser_data_frame"}, #melyik lidar frame-et hasznaljuk az akadalyok detektalasahoz
            ],
        ),
    ])
