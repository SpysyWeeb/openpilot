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
  2: (0.70, 0.30),
  3: (0.45, 0.20),
}
DEFAULT_LEVEL = 2



# settle-phase failsafe and smoothing
LINGER_SPEED = 0.3  # m/s, crawling below this without standstill arms the failsafe
LINGER_TIME = 1.0  # s, how long to crawl before the clamp is allowed to finish the stop
SETTLE_SMOOTH_SPEED = 1.5  # m/s, jerk-limit the PID output below this
SETTLE_JERK_LIMIT = 2.5  # m/s^3


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


class SmoothStopsLongControl:
  """Controls-side companion to SmoothStops, running at 100 Hz in longcontrol.

  Stock longcontrol enters the stopping state while the car is still rolling,
  which has two harsh consequences: the open-loop ramp adds brake pressure all
  the way into standstill, and on cars like HKG the stopping state asserts a
  stop request on the CAN bus, letting the factory brake controller run its own
  clamp procedure while still moving - whichever engages first wins, making the
  end of the stop a per-stop lottery.

  Defer the stopping state until the car is actually stationary. While rolling,
  the PID stays in charge - closed loop on measured acceleration - tracking the
  planner's landing law all the way down, adapting to slope, brake lag, and
  creep torque. The clamp and the stop request then land on a stopped car,
  where they cannot be felt. Stock behavior whenever Smooth Stops is disabled.
  """

  def __init__(self):
    self.params = Params()
    self.frame = 0
    self.enabled = False
    self.level = DEFAULT_LEVEL
    self.linger_frames = 0
    self.last_pid_output = 0.0

  def update(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_CTRL) == 0:
      self.enabled, self.level = read_smooth_stops_params(self.params)
    self.frame += 1

  def defer_stopping(self, should_stop: bool, standstill: bool, v_ego: float) -> bool:
    if not self.enabled:
      self.linger_frames = 0
      return should_stop

    if not should_stop or standstill:
      self.linger_frames = 0
      return should_stop

    # the plan wants a stop and the car is crawling: if the light settle brake
    # never closes the last bit to standstill, stop deferring and let the clamp
    # finish the stop. The car must never keep rolling when it should be stopped.
    if v_ego < LINGER_SPEED:
      self.linger_frames += 1
      if self.linger_frames >= int(LINGER_TIME / DT_CTRL):
        return True
    else:
      self.linger_frames = 0

    return False

  def smooth_pid_output(self, output_accel: float, v_ego: float) -> float:
    # jerk-limit the command in the settle regime to remove low-speed PID dither
    if not self.enabled or v_ego > SETTLE_SMOOTH_SPEED:
      self.last_pid_output = output_accel
      return output_accel

    step = SETTLE_JERK_LIMIT * DT_CTRL
    output_accel = min(max(output_accel, self.last_pid_output - step), self.last_pid_output + step)
    self.last_pid_output = output_accel
    return output_accel
