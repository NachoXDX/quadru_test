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
Fake gait: leg-swing joint offsets that make the cheat locomotion look like walking.

The body is actually pushed by the PlanarBodyDriver Gazebo plugin; these offsets
only animate the legs. Diagonal pairs (FL+RR, FR+RL) swing in anti-phase like a trot.
"""

import math

from quadruped_control.go2 import LEGS

# Phase offset of each leg in the gait cycle (radians): diagonal pairs move together.
LEG_PHASE = {'FL': 0.0, 'RR': 0.0, 'FR': math.pi, 'RL': math.pi}


def swing_offsets(phase, amplitude, stride=0.15, lift=0.2):
    """
    Return 12 joint offsets (JOINT_NAMES order) for the given gait phase.

    phase: gait clock in radians (one cycle = 2*pi).
    amplitude: 0..1 scale, 0 = standing still.
    stride: thigh swing amplitude (rad). Positive thigh angle moves the foot backwards.
    lift: thigh fold during swing (rad); the calf folds twice as much, which raises
        the foot while keeping it roughly under the hip (thigh and calf have equal length).
    """
    offsets = []
    for leg in LEGS:
        p = phase + LEG_PHASE[leg]
        # Swing for p in (0, pi): the foot moves back -> front while lifted.
        swing = max(0.0, math.sin(p))
        thigh = amplitude * (stride * math.cos(p) + lift * swing)
        calf = amplitude * (-2.0 * lift * swing)
        offsets.extend((0.0, thigh, calf))
    return offsets
