from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wayp_plan_tools',
            executable='elastic_band_avoidance',
            name='elastic_band_avoidance',
            output='screen',
            parameters=[
                # ── Input / output topics ─────────────────────────────────
                {"waypoint_topic": "waypointarray"},
                {"cloud_topic":    "pointcloud_obstacles"},
                {"output_topic":   "waypointarray_modified"},
                {"map_frame":      "map"},

                # ── Safety limits ─────────────────────────────────────────
                # Maximum allowed lateral displacement from the original
                # waypoint position (metres). Hard clamp – never exceeded.
                {"max_lateral_deviation": 9.0},

                # Minimum clearance enforced around each band node (m).
                # Acts as a singularity guard for the repulsion potential.
                {"robot_radius": 1.2},

                # ── Repulsion ─────────────────────────────────────────────
                # Obstacle points within this radius push band nodes away (m).
                {"influence_radius": 3.5},

                # Strength multiplier for the Khatib repulsion potential.
                # Increase if the robot clips obstacles; decrease if the path
                # deviates too aggressively.
                {"repulsion_gain": 3.0},

                # ── Spring (anchor pull-back) ─────────────────────────────
                # How strongly each node is pulled back to its original
                # position. Lower = more deviation; higher = stiffer path.
                {"spring_constant": 0.5},

                # ── Smoothness ────────────────────────────────────────────
                # Weight of the neighbour-averaging force that keeps the
                # deformed path smooth.
                {"smoothness_weight": 0.4},

                # ── Optimiser settings ────────────────────────────────────
                # Force-integration steps executed every 50 ms timer tick.
                {"eband_iterations": 8},

                # Number of waypoints ahead of the vehicle to process.
                {"lookahead_count": 60},

                # Fraction of remaining displacement that nodes outside the
                # active window snap back toward their anchor per 50 ms cycle.
                {"snapback_rate": 0.08},
            ],
        ),
    ])
