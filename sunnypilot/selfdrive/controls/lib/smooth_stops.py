"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Smooth Stops: ease off braking in the final moments of a stop.

A human driver feathers the brake as the car settles; openpilot can carry
1+ m/s^2 of deceleration all the way into standstill, rocking the car on its
suspension. Below a low speed threshold, limit how much braking the planner
may command, tapering to a gentle landing. The limit is bypassed whenever a
lead is close, so braking authority is never reduced when the gap demands it.
"""
import numpy as np

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD

SMOOTH_STOP_BP =        [0.0, 0.5, 1.5, 3.0]   # m/s
SMOOTH_STOP_MAX_BRAKE = [0.55, 0.65, 1.30, 3.50]  # m/s^2, max braking magnitude allowed
SMOOTH_STOP_MAX_SPEED = 3.0  # m/s, no effect above this speed
MIN_LEAD_DISTANCE = 4.0  # m, keep full braking authority when a lead is closer than this


class SmoothStops:
  def __init__(self):
    self.params = Params()
    self.frame = 0
    self.enabled = False
    self.active = False
    self.read_params()

  def read_params(self) -> None:
    self.enabled = self.params.get_bool("SmoothStops")

  def update(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.read_params()
    self.frame += 1

  def apply(self, a_target: float, v_ego: float, lead_one) -> float:
    self.active = False

    if not self.enabled or a_target >= 0. or v_ego > SMOOTH_STOP_MAX_SPEED:
      return a_target
    if lead_one.status and lead_one.dRel < MIN_LEAD_DISTANCE:
      return a_target

    brake_floor = -float(np.interp(v_ego, SMOOTH_STOP_BP, SMOOTH_STOP_MAX_BRAKE))
    if a_target < brake_floor:
      self.active = True
      a_target = brake_floor

    return a_target
