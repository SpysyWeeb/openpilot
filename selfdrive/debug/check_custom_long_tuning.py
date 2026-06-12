#!/usr/bin/env python3
"""Live monitor for sunnypilot custom longitudinal tuning.

Run on the device over SSH while driving (or with the car onroad in the driveway):
  cd /data/openpilot && python selfdrive/debug/check_custom_long_tuning.py

Change the profiles in Settings -> Cruise while this is running; the printed
values should update within ~3 seconds. maxAccel is the live cap from the
acceleration profile, decelJerkFactor shows the MPC jerk cost multiplier in effect.
"""
import time

import cereal.messaging as messaging


def main():
  sm = messaging.SubMaster(['carState', 'longitudinalPlan', 'longitudinalPlanSP', 'radarState', 'selfdriveState'])

  while True:
    sm.update(1000)

    cs = sm['carState']
    plan = sm['longitudinalPlan']
    plan_sp = sm['longitudinalPlanSP']
    tuning = plan_sp.customLongitudinalTuning
    lead = sm['radarState'].leadOne

    lead_txt = f"{lead.dRel:5.1f}m vLead {lead.vLead:4.1f}" if lead.status else "none"

    fields = [
      f"v {cs.vEgo * 2.237:5.1f}mph",
      f"aEgo {cs.aEgo:5.2f}",
      f"aTarget {plan.aTarget:5.2f}",
      f"custom {int(tuning.enabled)} maxAccel {tuning.maxAccel:4.2f} jerkF {tuning.jerkFactor:4.2f} tFollow {tuning.tFollow:4.2f}",
      f"exp {int(sm['selfdriveState'].experimentalMode)} decActive {int(plan_sp.dec.active)}",
      f"lead {lead_txt}",
    ]
    print(" | ".join(fields))

    time.sleep(0.5)


if __name__ == "__main__":
  main()
