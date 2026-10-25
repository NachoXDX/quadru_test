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

"""Go2 joint layout and reference poses shared by the control nodes."""

LEGS = ('FL', 'FR', 'RL', 'RR')

# Same order as forward_position_controller's `joints` in
# quadruped_bringup/config/controllers.yaml.
JOINT_NAMES = tuple(
    f'{leg}_{joint}_joint' for leg in LEGS for joint in ('hip', 'thigh', 'calf'))


def _same_for_all_legs(hip, thigh, calf):
    return (hip, thigh, calf) * len(LEGS)


# Joint angles in radians, in JOINT_NAMES order.
LIE_POSE = _same_for_all_legs(0.0, 1.36, -2.65)    # belly on the ground, legs folded
STAND_POSE = _same_for_all_legs(0.0, 0.8, -1.5)    # trunk ~0.33 m above the ground
