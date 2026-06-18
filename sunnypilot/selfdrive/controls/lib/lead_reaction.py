"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Lead Reaction and Launch Response: two global longitudinal-feel knobs that
default to stock behavior and are read independently of any other tuning.

- Lead Reaction scales the lead's acceleration-decay time constant (aLeadTau): a
  higher reactivity divides aLeadTau down, so the MPC projects the lead's braking
  (or pull-away) as persisting longer and engages the response earlier and more
  progressively instead of late and abrupt.
- Launch Response scales the v_desired smoothing time constant used at takeoff: a
  higher quickness divides the stock 2.0 s tau down, tightening the tracking of
  the model's pull-off from a stop. The planner blends this back to stock above a
  low speed so highway following is untouched.

Both sliders are stored as ints in 0.1x steps; 1.0x reproduces stock exactly.
"""
import numpy as np

from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL
from openpilot.sunnypilot import PARAMS_UPDATE_PERIOD

REACTION_PARAM_SCALE = 0.1
REACTION_MIN, REACTION_MAX = 1.0, 5.0

STOCK_V_DESIRED_TAU = 2.0
LAUNCH_PARAM_SCALE = 0.1
LAUNCH_MIN, LAUNCH_MAX = 1.0, 4.0


class LeadReactionTuning:
  def __init__(self):
    self.params = Params()
    self.frame = 0
    self.lead_reaction_factor = 1.0
    self.launch_tau = STOCK_V_DESIRED_TAU
    self.read_params()

  def read_params(self) -> None:
    reactivity = int(self.params.get("LeadReaction", return_default=True)) * REACTION_PARAM_SCALE
    reactivity = float(np.clip(reactivity, REACTION_MIN, REACTION_MAX))
    self.lead_reaction_factor = 1.0 / reactivity

    quickness = int(self.params.get("LaunchResponse", return_default=True)) * LAUNCH_PARAM_SCALE
    quickness = float(np.clip(quickness, LAUNCH_MIN, LAUNCH_MAX))
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
