"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Reaction Level: one global longitudinal-feel knob, a whole number from 1 (stock)
to 5 (quickest), that makes sunnypilot respond quicker on both gas and brakes.
It drives two effects together, interpolated across the level:

- Lead reaction scales the lead's acceleration-decay time constant (aLeadTau): a
  higher level divides aLeadTau down, so the MPC projects the lead's braking (or
  pull-away) as persisting longer and engages the response earlier and more
  progressively instead of late and abrupt. At level 5 this is 5x stock.
- Launch response scales the v_desired smoothing time constant used at takeoff: a
  higher level divides the stock 2.0 s tau down, tightening the tracking of the
  model's pull-off from a stop. At level 5 this is 4x stock. The planner blends
  this back to stock above a low speed so highway following is untouched.

Level 1 reproduces stock behavior exactly.
"""
import numpy as np

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD

REACTION_MIN_LEVEL, REACTION_MAX_LEVEL = 1, 5

# at the top level, lead reaction is 5x stock and launch response is 4x stock
REACTIVITY_AT_MAX = 5.0
LAUNCH_QUICKNESS_AT_MAX = 4.0
STOCK_V_DESIRED_TAU = 2.0


class LeadReactionTuning:
  def __init__(self):
    self.params = Params()
    self.frame = 0
    self.lead_reaction_factor = 1.0
    self.launch_tau = STOCK_V_DESIRED_TAU
    self.read_params()

  def read_params(self) -> None:
    level = float(np.clip(int(self.params.get("ReactionLevel", return_default=True)),
                          REACTION_MIN_LEVEL, REACTION_MAX_LEVEL))

    reactivity = np.interp(level, [REACTION_MIN_LEVEL, REACTION_MAX_LEVEL], [1.0, REACTIVITY_AT_MAX])
    self.lead_reaction_factor = 1.0 / reactivity

    quickness = np.interp(level, [REACTION_MIN_LEVEL, REACTION_MAX_LEVEL], [1.0, LAUNCH_QUICKNESS_AT_MAX])
    self.launch_tau = STOCK_V_DESIRED_TAU / quickness

  def update(self) -> None:
    if self.frame % int(PARAMS_UPDATE_PERIOD / DT_MDL) == 0:
      self.read_params()
    self.frame += 1

  def get_lead_reaction_factor(self) -> float:
    # multiplier applied to the lead's aLeadTau in the MPC; 1.0 is stock
    return self.lead_reaction_factor

  def get_launch_tau(self) -> float:
    # target v_desired smoothing time constant at takeoff; stock is STOCK_V_DESIRED_TAU
    return self.launch_tau
