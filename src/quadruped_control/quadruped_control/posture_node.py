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
Posture node: moves the Go2 between lying and standing.

Holds the measured pose at startup, then on /stand_up or /lie_down interpolates
all 12 joints to the target pose and streams the setpoints to
forward_position_controller.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger

from quadruped_control.go2 import JOINT_NAMES, LIE_POSE, STAND_POSE


def smoothstep(s):
    """Ease-in/ease-out from 0 to 1: zero velocity at both ends, so no jerk."""
    s = min(max(s, 0.0), 1.0)
    return s * s * (3.0 - 2.0 * s)


class PostureNode(Node):

    def __init__(self):
        super().__init__('posture_node')
        self.declare_parameter('transition_time', 2.0)  # s
        self.declare_parameter('rate', 100.0)           # Hz

        self._command = None   # last commanded pose; None until joint states arrive
        self._start = None     # pose at the beginning of the current transition
        self._target = None
        self._t0 = None

        self._pub = self.create_publisher(
            Float64MultiArray, '/forward_position_controller/commands', 10)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        self.create_service(Trigger, 'stand_up', self._make_handler(STAND_POSE, 'standing up'))
        self.create_service(Trigger, 'lie_down', self._make_handler(LIE_POSE, 'lying down'))
        self.create_timer(1.0 / self.get_parameter('rate').value, self._on_timer)

    def _on_joint_states(self, msg):
        if self._command is not None:
            return
        positions = dict(zip(msg.name, msg.position))
        if not all(name in positions for name in JOINT_NAMES):
            return
        # First complete reading: hold where the robot is, so activation doesn't jump.
        self._command = [positions[name] for name in JOINT_NAMES]
        self.get_logger().info('Joint states received, holding current pose. '
                               'Call /stand_up or /lie_down.')

    def _make_handler(self, pose, description):
        def handler(request, response):
            if self._command is None:
                response.success = False
                response.message = 'No joint states yet'
                return response
            self._start = list(self._command)
            self._target = list(pose)
            self._t0 = self.get_clock().now()
            response.success = True
            response.message = description
            self.get_logger().info(description.capitalize())
            return response
        return handler

    def _on_timer(self):
        if self._command is None:
            return
        if self._target is not None:
            elapsed = (self.get_clock().now() - self._t0).nanoseconds * 1e-9
            s = smoothstep(elapsed / self.get_parameter('transition_time').value)
            self._command = [a + s * (b - a) for a, b in zip(self._start, self._target)]
            if s >= 1.0:
                self._target = None
        self._pub.publish(Float64MultiArray(data=self._command))


def main(args=None):
    try:
        with rclpy.init(args=args):
            rclpy.spin(PostureNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
