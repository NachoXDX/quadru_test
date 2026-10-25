# Copyright 2026 Nacho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Noisy odometry: dead reckoning that drifts like real wheel or leg odometry.

Takes the true planar motion between two ground-truth poses (forward/sideways
velocity in the heading frame and yaw rate), corrupts it with a constant scale
error, a yaw-rate bias and white noise, integrates it, and publishes the result as
/odom plus the odom -> base_link transform. Position and yaw therefore drift
without bound, which is what the EKF (Stage 5) and SLAM (Stage 6) correct.
Height, roll and pitch are copied from ground truth: a planar odometry source
doesn't observe them, and the IMU takes that role later.
"""

import math
import random

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(q):
    """Yaw (rad) of a quaternion."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def roll_pitch_from_quaternion(q):
    """Roll and pitch (rad) of a quaternion."""
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


def quaternion_from_rpy(roll, pitch, yaw):
    """Quaternion from roll, pitch, yaw (rad)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy)


class NoisyOdomNode(Node):

    def __init__(self):
        super().__init__('noisy_odom_node')
        self.declare_parameter('scale_error', 0.03)     # +3 % on linear velocity
        self.declare_parameter('yaw_rate_bias', 0.01)   # rad/s, like an uncalibrated gyro
        self.declare_parameter('linear_noise', 0.02)    # m/s, std dev of white noise
        self.declare_parameter('angular_noise', 0.02)   # rad/s, std dev of white noise
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('seed', 0)               # 0 = random each run

        seed = self.get_parameter('seed').value
        self._rng = random.Random(seed or None)
        self._x = self._y = self._yaw = 0.0
        self._last_stamp = None
        self._last_truth = None   # (x, y, yaw) of the previous ground-truth message

        self._pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf = TransformBroadcaster(self) if self.get_parameter('publish_tf').value else None
        self.create_subscription(Odometry, '/ground_truth/odom', self._on_ground_truth, 10)

    def _on_ground_truth(self, truth):
        stamp = truth.header.stamp.sec + truth.header.stamp.nanosec * 1e-9
        position = truth.pose.pose.position
        truth_yaw = yaw_from_quaternion(truth.pose.pose.orientation)
        if self._last_stamp is None:
            # Start where the robot really is, like resetting odometry at boot.
            self._x, self._y, self._yaw = position.x, position.y, truth_yaw
            self._last_stamp, self._last_truth = stamp, (position.x, position.y, truth_yaw)
            return
        dt = stamp - self._last_stamp
        if dt <= 0.0:
            return
        last_x, last_y, last_yaw = self._last_truth
        self._last_stamp, self._last_truth = stamp, (position.x, position.y, truth_yaw)

        # True planar motion since the last message, in the heading (yaw-only) frame.
        # (The body-frame twist would be wrong here: while the body rocks, its
        # z angular rate is not the yaw rate, and the error builds up into drift.)
        dx, dy = position.x - last_x, position.y - last_y
        true_vx = (dx * math.cos(last_yaw) + dy * math.sin(last_yaw)) / dt
        true_vy = (-dx * math.sin(last_yaw) + dy * math.cos(last_yaw)) / dt
        true_wz = math.atan2(math.sin(truth_yaw - last_yaw), math.cos(truth_yaw - last_yaw)) / dt

        scale = 1.0 + self.get_parameter('scale_error').value
        lin_noise = self.get_parameter('linear_noise').value
        ang_noise = self.get_parameter('angular_noise').value
        # What an imperfect odometry sensor reports.
        vx = true_vx * scale + self._rng.gauss(0.0, lin_noise)
        vy = true_vy * scale + self._rng.gauss(0.0, lin_noise)
        wz = true_wz + self.get_parameter('yaw_rate_bias').value + self._rng.gauss(0.0, ang_noise)

        # Dead reckoning: integrate in the odom frame (velocity was measured
        # in the heading at the start of the interval).
        heading = self._yaw
        self._x += (vx * math.cos(heading) - vy * math.sin(heading)) * dt
        self._y += (vx * math.sin(heading) + vy * math.cos(heading)) * dt
        self._yaw += wz * dt

        roll, pitch = roll_pitch_from_quaternion(truth.pose.pose.orientation)
        orientation = quaternion_from_rpy(roll, pitch, self._yaw)

        odom = Odometry()
        odom.header.stamp = truth.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.position.z = truth.pose.pose.position.z
        odom.pose.pose.orientation = orientation
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        # Twist covariance = the noise we inject (diagonal: x, y, z, roll, pitch, yaw).
        lin_var, ang_var = lin_noise ** 2, ang_noise ** 2
        odom.twist.covariance[0] = odom.twist.covariance[7] = lin_var
        odom.twist.covariance[35] = ang_var
        self._pub.publish(odom)

        if self._tf is not None:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = 'base_link'
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.translation.z = truth.pose.pose.position.z
            t.transform.rotation = orientation
            self._tf.sendTransform(t)


def main(args=None):
    try:
        with rclpy.init(args=args):
            rclpy.spin(NoisyOdomNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
