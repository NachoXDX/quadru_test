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
Joystick actions: map gamepad buttons to robot services.

Default mapping (SDL game controller layout, as published by joy's
game_controller_node): button 0 (PS5 Cross) -> /stand_up,
button 1 (PS5 Circle) -> /lie_down. Fires on the press, not while held.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger


class JoyActionsNode(Node):

    def __init__(self):
        super().__init__('joy_actions_node')
        self.declare_parameter('stand_button', 0)
        self.declare_parameter('lie_button', 1)
        self._actions = {
            self.get_parameter('stand_button').value: self.create_client(Trigger, 'stand_up'),
            self.get_parameter('lie_button').value: self.create_client(Trigger, 'lie_down'),
        }
        self._previous = []
        self.create_subscription(Joy, '/joy', self._on_joy, 10)

    def _on_joy(self, msg):
        for button, client in self._actions.items():
            pressed = button < len(msg.buttons) and msg.buttons[button]
            was_pressed = button < len(self._previous) and self._previous[button]
            if pressed and not was_pressed:
                if client.service_is_ready():
                    client.call_async(Trigger.Request())
                else:
                    self.get_logger().warn(f'{client.srv_name} not available')
        self._previous = list(msg.buttons)


def main(args=None):
    try:
        with rclpy.init(args=args):
            rclpy.spin(JoyActionsNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
