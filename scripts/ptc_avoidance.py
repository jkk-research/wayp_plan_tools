#!/usr/bin/env python3
import copy, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class PtcAvoidance(Node):
    def __init__(self):
        super().__init__('ptc_avoidance')
        self.declare_parameter('corridor_width',      1.5)    # danger half-width (m)
        self.declare_parameter('min_obstacle_points', 5)      # hits needed to avoid
        self.declare_parameter('offset_distance',     2.0)    # lateral shift (m)
        self.declare_parameter('avoidance_direction', 'left') # left / right / auto
        self.declare_parameter('lookahead_count',     20)     # waypoints to check ahead
        self.traj = None
        self.create_subscription(PoseArray,   'targetpoints',         self._traj_cb,  10)
        self.create_subscription(PointCloud2, 'pointcloud_obstacles', self._cloud_cb, 10)
        self.pub = self.create_publisher(PoseArray, 'targetpoints_modified', 10)

    @staticmethod
    def _seg_dist(px, py, ax, ay, bx, by):
        """Shortest distance from point P to segment AB."""
        dx, dy = bx-ax, by-ay
        l2 = dx*dx + dy*dy
        if l2 < 1e-9: return math.hypot(px-ax, py-ay)
        t = max(0.0, min(1.0, ((px-ax)*dx + (py-ay)*dy) / l2))
        return math.hypot(px-ax-t*dx, py-ay-t*dy)

    @staticmethod
    def _cross2d(px, py, ax, ay, bx, by):   # +ve → P is left of A→B
        return (bx-ax)*(py-ay) - (by-ay)*(px-ax)

    def _traj_cb(self, msg): self.traj = msg

    def _cloud_cb(self, msg):
        if not self.traj or not self.traj.poses:
            return
        cw   = self.get_parameter('corridor_width').value
        mop  = self.get_parameter('min_obstacle_points').value
        od   = self.get_parameter('offset_distance').value
        adir = self.get_parameter('avoidance_direction').value
        n    = min(self.get_parameter('lookahead_count').value, len(self.traj.poses))
        poses = self.traj.poses

        # 1. collect cloud points inside the trajectory corridor
        hits = []
        for pt in point_cloud2.read_points(msg, field_names=('x', 'y'), skip_nans=True):
            px, py = float(pt[0]), float(pt[1])
            for i in range(n - 1):
                ax, ay = poses[i].position.x,   poses[i].position.y
                bx, by = poses[i+1].position.x, poses[i+1].position.y
                if self._seg_dist(px, py, ax, ay, bx, by) <= cw:
                    hits.append((px, py)); break

        out = copy.deepcopy(self.traj)
        out.header.stamp = self.get_clock().now().to_msg()

        # 2. when threshold is reached, shift all waypoints laterally
        if len(hits) >= mop:
            self.get_logger().warn(f'{len(hits)} pts in corridor – avoiding',
                                   throttle_duration_sec=1.0)
            sign = -1.0 if adir == 'right' else 1.0
            if adir == 'auto':
                cx = sum(h[0] for h in hits) / len(hits)
                cy = sum(h[1] for h in hits) / len(hits)
                c  = self._cross2d(cx, cy, poses[0].position.x, poses[0].position.y,
                                           poses[1].position.x, poses[1].position.y)
                sign = -1.0 if c > 0 else 1.0   # obstacle on left → go right

            n_all = len(out.poses)
            for i in range(n_all):
                if i < n_all - 1:
                    tx = out.poses[i+1].position.x - out.poses[i].position.x
                    ty = out.poses[i+1].position.y - out.poses[i].position.y
                else:
                    tx = out.poses[i].position.x - out.poses[i-1].position.x
                    ty = out.poses[i].position.y - out.poses[i-1].position.y
                ln = math.hypot(tx, ty)
                if ln < 1e-6: continue
                nx, ny = -ty/ln, tx/ln           # left-perpendicular normal
                out.poses[i].position.x += sign * od * nx
                out.poses[i].position.y += sign * od * ny

        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(PtcAvoidance())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
