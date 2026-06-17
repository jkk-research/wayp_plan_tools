from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wayp_plan_tools',
            executable='pointcloud_trajectory_avoidance',
            name='pointcloud_trajectory_avoidance',
            output='screen',
            parameters=[
                # ── Input / output topics ─────────────────────────────────
                # Local trajectory (geometry_msgs/PoseArray in vehicle frame)
                {"trajectory_topic": "targetpoints"},
                # PointCloud2 obstacle cloud (should be in the same frame as
                # the trajectory, typically the vehicle's base_link frame)
                {"cloud_topic":      "pointcloud_obstacles"},
                # Modified trajectory output
                {"output_topic":     "targetpoints_modified"},

                # ── Corridor detection ────────────────────────────────────
                # Half-width of the safety corridor checked around the path (m).
                # A cloud point within this distance of any trajectory segment
                # is counted as an obstacle.
                {"corridor_width": 1.5},

                # Number of obstacle points required inside the corridor to
                # trigger avoidance.  Increase to reduce false positives.
                {"min_obstacle_points": 5},

                # Number of waypoints ahead (from index 0) to check for
                # obstacles.  Keep this ≤ the length of the local trajectory.
                {"lookahead_count": 20},

                # ── Avoidance offset ──────────────────────────────────────
                # Lateral offset applied to every waypoint when avoidance is
                # active (m).
                {"offset_distance": 2.0},

                # "left"  – always offset to the left of the travel direction
                # "right" – always offset to the right
                # "auto"  – automatically offset away from the obstacle centroid
                {"avoidance_direction": "auto"},
            ],
        ),
    ])
