"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Smooth Stops: exponential-landing stops.

A human driver feathers the brake as the car settles; openpilot can carry
1+ m/s^2 of deceleration all the way into standstill, rocking the car on its
suspension. A velocity profile that "comes in for a smooth landing" is
exponential decay, produced by capping deceleration in proportion to speed:

  a_allowed = -(k * v + c)

k sets the landing's time constant and is the user-facing smoothness level;
c keeps the stop from asymptoting forever and is the residual deceleration at
standstill (the size of the remaining head-nod). Near the activation speed the
cap computes to ~full braking, so the approach can be as firm as the situation
demands and blends seamlessly into the landing.

Only applied when the planner's own trajectory comes to a stop, so braking for
corners and slowdowns is untouched, and bypassed whenever a lead is close so
braking authority is never reduced when the gap demands it.
"""
from openpilot.common.params import Params
from openpilot.common.realtime import DT_CTRL, DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD

ACTIVATION_SPEED = 3.5  # m/s, cap is computed below this; near no-op at the top end
STOP_INTENT_SPEED = 0.5  # m/s, plan must reach below this to count as a stop
MIN_LEAD_DISTANCE = 4.0  # m, keep full braking authority when a lead is closer than this

# level -> (k [1/s], c [m/s^2])
SMOOTHNESS_LEVELS = {
  1: (1.10, 0.40),
  2: (0.90, 0.35),
  3: (0.70, 0.30),
  4: (0.55, 0.25),
  5: (0.45, 0.20),
}
DEFAULT_LEVEL = 3

# level -> scale on longcontrol's stopping-state brake ramp (CP.stoppingDecelRate)
STOPPING_DECEL_RATE_SCALE = {1: 0.60, 2: 0.50, 3: 0.40, 4: 0.30, 5: 0.25}


def read_smooth_stops_params(params: Params) -> tuple[bool, int]:
  enabled = params.get_bool("SmoothStops")
  level = int(params.get("SmoothStopsLevel", return_default=True))
  return enabled, min(max(level, min(SMOOTHNESS_LEVELS)), max(SMOOTHNESS_LEVELS))


class SmoothStops:
  def __init__(self):
    self.params = Params()
    self.frame = 0
    self.enabled = False
    self.level = DEFAULT_LEVEL
    self.active = False
    self.read_params()

  def read_params(self) -> None:
    self.enabled, self.level = read_smooth_stops_params(self.params)

  def update(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.read_params()
    self.frame += 1

  def apply(self, a_target: float, v_ego: float, lead_one, plan_min_v: float) -> float:
    self.active = False

    if not self.enabled or a_target >= 0. or v_ego > ACTIVATION_SPEED:
      return a_target
    if plan_min_v > STOP_INTENT_SPEED:
      return a_target
    if lead_one.status and lead_one.dRel < MIN_LEAD_DISTANCE:
      return a_target

    k, c = SMOOTHNESS_LEVELS[self.level]
    brake_floor = -(k * v_ego + c)
    if a_target < brake_floor:
      self.active = True
      a_target = brake_floor

    return a_target


class SmoothStopsStoppingRamp:
  """Controls-side companion to SmoothStops, running at 100 Hz in longcontrol.

  longcontrol's stopping state ignores the planner's aTarget and ramps the brake
  command toward CP.stopAccel at CP.stoppingDecelRate; that ramp is what the car
  feels at the moment of physical standstill. Slowing the ramp lands the stop
  gently while the full stopAccel hold still arrives - just after the car is
  already stationary. Stock behavior whenever Smooth Stops is disabled.
  """

  def __init__(self):
    self.params = Params()
    self.frame = 0
    self.enabled = False
    self.level = DEFAULT_LEVEL

  def update(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_CTRL) == 0:
      self.enabled, self.level = read_smooth_stops_params(self.params)
    self.frame += 1

  def get_stopping_decel_rate(self, stock_rate: float) -> float:
    if not self.enabled:
      return stock_rate
    return stock_rate * STOPPING_DECEL_RATE_SCALE[self.level]
