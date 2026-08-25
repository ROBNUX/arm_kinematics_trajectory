#!/usr/bin/env python3
"""
Derive joint velocity/acceleration and Cartesian velocity/acceleration by
finite-differencing the position-only topics every simulate_*.py script
already publishes, and republish them so they can be rosbag-recorded
alongside the originals.

None of robnux_arm_sim's simulation nodes publish velocity or acceleration
today: /joint_states carries position only (velocity/effort are left
empty), and /cart_pose carries position/orientation only. The trajectory
engine computes joint velocity/acceleration internally during execution but
never exposes them over ROS, and no Cartesian velocity/acceleration is
computed anywhere in the C++ stack. Rather than change the trajectory
library's hot path to expose that internal state (touching the
recently-fixed scurve_lib/trajectory core for an unrelated feature), this
node differentiates the public, already-correct position stream instead —
zero risk to the trajectory engine, at the cost of finite-difference noise
in acceleration (a double derivative). Good enough for offline debugging
and replay; not intended as a control-loop feedback source.

Subscribes:
    /joint_states   sensor_msgs/JointState      (position only)
    /cart_pose      geometry_msgs/PoseStamped   (position + orientation)

Publishes:
    /joint_states_full   sensor_msgs/JointState
        position = passthrough from /joint_states
        velocity = d(position)/dt per joint (matched by name)
        effort   = d(velocity)/dt per joint, i.e. ACCELERATION, not torque
                   (JointState has no dedicated acceleration field; effort
                   is otherwise always empty on this topic, so it's
                   repurposed here -- see the field comment below)
    /cart_twist   geometry_msgs/TwistStamped
        linear  = d(position)/dt
        angular = instantaneous angular velocity from consecutive
                  orientation quaternions (shortest-arc axis-angle / dt,
                  not a naive Euler-angle rate -- avoids gimbal-lock noise)
    /cart_accel   geometry_msgs/AccelStamped
        linear/angular = d(/cart_twist)/dt

Run alongside any simulate_*.py (order doesn't matter, it just waits for
messages):
    ros2 run robnux_arm_sim trajectory_recorder.py
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import AccelStamped, PoseStamped, TwistStamped
from sensor_msgs.msg import JointState

MIN_DT = 1e-4  # below this, treat as a duplicate/out-of-order stamp and skip


def _stamp_to_sec(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


class _JointDiff:
    """Per-joint position -> velocity -> acceleration, matched by name."""

    def __init__(self):
        self.prev_t = None
        self.prev_pos = {}   # name -> position
        self.prev_vel = {}   # name -> velocity

    def update(self, t: float, names, positions):
        vel = {}
        accel = {}
        dt = None if self.prev_t is None else t - self.prev_t
        for name, pos in zip(names, positions):
            if dt is not None and dt > MIN_DT and name in self.prev_pos:
                delta = pos - self.prev_pos[name]
                # A revolute joint's reported position wraps at +-pi, so a
                # step crossing that boundary (e.g. 3.14 -> -3.14) is a tiny
                # real motion, not a near-2*pi jump -- unwrap it to the
                # equivalent shortest-path delta. A genuine delta this large
                # in one ~2ms control cycle would be physically absurd for
                # any joint (revolute or prismatic) regardless, so applying
                # this unconditionally is safe rather than gating it on
                # joint type.
                if delta > math.pi:
                    delta -= 2.0 * math.pi
                elif delta < -math.pi:
                    delta += 2.0 * math.pi
                v = delta / dt
                vel[name] = v
                if name in self.prev_vel:
                    accel[name] = (v - self.prev_vel[name]) / dt
            self.prev_pos[name] = pos
        if dt is not None and dt > MIN_DT:
            self.prev_vel = vel
            self.prev_t = t
        elif self.prev_t is None:
            self.prev_t = t
        return vel, accel


class _CartDiff:
    """Cartesian position/orientation -> twist -> accel."""

    def __init__(self):
        self.prev_t = None
        self.prev_pos = None       # (x,y,z)
        self.prev_quat = None      # (w,x,y,z)
        self.prev_twist = None     # (lx,ly,lz, ax,ay,az)

    @staticmethod
    def _angular_vel(q_prev, q_curr, dt):
        # dq = q_curr * conjugate(q_prev), rotation from prev frame to
        # curr frame expressed in the (shared) world frame.
        w0, x0, y0, z0 = q_prev
        w1, x1, y1, z1 = q_curr
        # conjugate(q_prev) = (w0,-x0,-y0,-z0); dq = q1 * conj(q0)
        cw, cx, cy, cz = w0, -x0, -y0, -z0
        dw = w1 * cw - x1 * cx - y1 * cy - z1 * cz
        dx = w1 * cx + x1 * cw + y1 * cz - z1 * cy
        dy = w1 * cy - x1 * cz + y1 * cw + z1 * cx
        dz = w1 * cz + x1 * cy - y1 * cx + z1 * cw
        norm = math.sqrt(dw * dw + dx * dx + dy * dy + dz * dz)
        if norm < 1e-12:
            return 0.0, 0.0, 0.0
        dw, dx, dy, dz = dw / norm, dx / norm, dy / norm, dz / norm
        if dw < 0.0:  # take the shortest-arc representative
            dw, dx, dy, dz = -dw, -dx, -dy, -dz
        dw = max(-1.0, min(1.0, dw))
        angle = 2.0 * math.acos(dw)
        s = math.sqrt(max(0.0, 1.0 - dw * dw))
        if s < 1e-9:
            return 0.0, 0.0, 0.0
        return (dx / s) * angle / dt, (dy / s) * angle / dt, (dz / s) * angle / dt

    def update(self, t: float, pos, quat):
        dt = None if self.prev_t is None else t - self.prev_t
        twist = None
        accel = None
        if dt is not None and dt > MIN_DT:
            lx = (pos[0] - self.prev_pos[0]) / dt
            ly = (pos[1] - self.prev_pos[1]) / dt
            lz = (pos[2] - self.prev_pos[2]) / dt
            ax, ay, az = self._angular_vel(self.prev_quat, quat, dt)
            twist = (lx, ly, lz, ax, ay, az)
            if self.prev_twist is not None:
                accel = tuple((twist[i] - self.prev_twist[i]) / dt for i in range(6))
            self.prev_twist = twist
            self.prev_t = t
        elif self.prev_t is None:
            self.prev_t = t
        self.prev_pos = pos
        self.prev_quat = quat
        return twist, accel


class TrajectoryRecorder(Node):
    def __init__(self):
        super().__init__("trajectory_recorder")
        self._jd = _JointDiff()
        self._cd = _CartDiff()

        self.create_subscription(JointState, "/joint_states", self._on_joint, 200)
        self.create_subscription(PoseStamped, "/cart_pose", self._on_pose, 200)

        self._joint_pub = self.create_publisher(JointState, "/joint_states_full", 200)
        self._twist_pub = self.create_publisher(TwistStamped, "/cart_twist", 200)
        self._accel_pub = self.create_publisher(AccelStamped, "/cart_accel", 200)

        self.get_logger().info(
            "trajectory_recorder: differentiating /joint_states + /cart_pose -> "
            "/joint_states_full, /cart_twist, /cart_accel"
        )

    def _on_joint(self, msg: JointState):
        t = _stamp_to_sec(msg.header.stamp)
        vel, accel = self._jd.update(t, msg.name, msg.position)
        out = JointState()
        out.header = msg.header
        out.name = list(msg.name)
        out.position = list(msg.position)
        out.velocity = [vel.get(n, 0.0) for n in msg.name]
        out.effort = [accel.get(n, 0.0) for n in msg.name]  # repurposed: acceleration
        self._joint_pub.publish(out)

    def _on_pose(self, msg: PoseStamped):
        t = _stamp_to_sec(msg.header.stamp)
        p = msg.pose.position
        q = msg.pose.orientation
        twist, accel = self._cd.update(t, (p.x, p.y, p.z), (q.w, q.x, q.y, q.z))
        if twist is not None:
            tw = TwistStamped()
            tw.header = msg.header
            tw.twist.linear.x, tw.twist.linear.y, tw.twist.linear.z = twist[0:3]
            tw.twist.angular.x, tw.twist.angular.y, tw.twist.angular.z = twist[3:6]
            self._twist_pub.publish(tw)
        if accel is not None:
            ac = AccelStamped()
            ac.header = msg.header
            ac.accel.linear.x, ac.accel.linear.y, ac.accel.linear.z = accel[0:3]
            ac.accel.angular.x, ac.accel.angular.y, ac.accel.angular.z = accel[3:6]
            self._accel_pub.publish(ac)


def main() -> int:
    rclpy.init()
    node = TrajectoryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
