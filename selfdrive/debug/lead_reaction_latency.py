#!/usr/bin/env python3
"""
Lead-reaction latency decomposition.

Given a logged route, estimate WHERE the delay between a lead vehicle's
acceleration change and our response actually comes from. It measures the time
lag between each stage of the longitudinal pipeline by cross-correlating their
deceleration dynamics during lead-following:

  model lead accel   modelV2.leadsV3[0].a[0]      anticipatory; can see brake lights
   -> vLead deriv    d/dt radarState.leadOne.vLead  radar kinematics (raw, low-lag)
   -> aLeadK         radarState.leadOne.aLeadK      Kalman-smoothed accel the MPC uses
   -> plan aTarget   longitudinalPlan.aTarget       planner output
   -> cmd accel      carControl.actuators.accel     commanded accel
   -> vehicle aEgo   carState.aEgo                  actual vehicle response

The point: radar reports range/velocity fast, but its *acceleration* is a
smoothed derivative, and the felt delay may live downstream (control/actuation)
rather than in perception. Find the binding stage before tuning anything.

Positive lag = the downstream signal trails the upstream one by that many ms.
A low correlation (< ~0.3) means the estimate is unreliable for that route.

Usage:
  python selfdrive/debug/lead_reaction_latency.py "<route_or_segment_or_rlog>"
  # Tip: append '/a' to a route name to fall back to qlogs if rlogs aren't up.
"""
import sys
import numpy as np

from openpilot.tools.lib.logreader import LogReader

DT = 0.01               # resample grid: 100 Hz
MAX_LAG_S = 2.0         # search lags in [0, 2.0] s
MIN_LEAD_PROB = 0.5     # model lead confidence gate
FOLLOW_MIN_VEGO = 2.0   # m/s; ignore near-standstill
MIN_CORR = 0.30         # below this, flag the lag as unreliable
DECEL_EVENT = -1.2      # m/s^2; aEgo below this marks a braking event
EVENT_PAD = 2.0         # s; window around each event to cross-correlate within

PLAN_SOURCE = {0: "cruise", 1: "lead0", 2: "lead1", 3: "lead2", 4: "e2e"}


def collect(route):
  cont = {k: ([], []) for k in ("model_a", "vlead", "aleadk", "atarget", "cmd", "aego", "vego",
                                "has_lead", "longactive", "brake", "gas")}
  plan_src = []
  for msg in LogReader(route):
    t = msg.logMonoTime * 1e-9
    w = msg.which()
    if w == "modelV2":
      leads = msg.modelV2.leadsV3
      if len(leads) and leads[0].prob > MIN_LEAD_PROB and len(leads[0].a):
        cont["model_a"][0].append(t); cont["model_a"][1].append(float(leads[0].a[0]))
    elif w == "radarState":
      lo = msg.radarState.leadOne
      if lo.status:
        cont["vlead"][0].append(t);  cont["vlead"][1].append(float(lo.vLead))
        cont["aleadk"][0].append(t); cont["aleadk"][1].append(float(lo.aLeadK))
    elif w == "longitudinalPlan":
      lp = msg.longitudinalPlan
      cont["atarget"][0].append(t);  cont["atarget"][1].append(float(lp.aTarget))
      cont["has_lead"][0].append(t); cont["has_lead"][1].append(float(lp.hasLead))
      plan_src.append(int(lp.longitudinalPlanSource.raw))
    elif w == "carControl":
      cont["cmd"][0].append(t); cont["cmd"][1].append(float(msg.carControl.actuators.accel))
      cont["longactive"][0].append(t); cont["longactive"][1].append(float(msg.carControl.longActive))
    elif w == "carState":
      cs = msg.carState
      cont["aego"][0].append(t); cont["aego"][1].append(float(cs.aEgo))
      cont["vego"][0].append(t); cont["vego"][1].append(float(cs.vEgo))
      cont["brake"][0].append(t); cont["brake"][1].append(float(cs.brakePressed))
      cont["gas"][0].append(t);   cont["gas"][1].append(float(cs.gasPressed))
  return cont, plan_src


def resample(cont):
  cores = [k for k in ("aleadk", "atarget", "cmd", "aego") if cont[k][0]]
  if not cores:
    raise SystemExit("no usable longitudinal signals in route (need carState/carControl/longitudinalPlan)")
  t0 = max(min(cont[k][0]) for k in cores)
  t1 = min(max(cont[k][0]) for k in cores)
  grid = np.arange(t0, t1, DT)
  out = {"t": grid}
  for k, (ts, vs) in cont.items():
    if not ts:
      out[k] = np.full(grid.shape, np.nan)
      continue
    ts, vs = np.array(ts), np.array(vs)
    order = np.argsort(ts)
    out[k] = np.interp(grid, ts[order], vs[order], left=np.nan, right=np.nan)
  out["vlead_deriv"] = np.gradient(out["vlead"], DT) if cont["vlead"][0] else np.full(grid.shape, np.nan)
  return out


def detrend(x, win_s=3.0):
  k = max(1, int(win_s / DT))
  base = np.convolve(np.nan_to_num(x), np.ones(k) / k, mode="same")
  return x - base


def smooth(x, win_s):
  k = max(1, int(win_s / DT))
  return np.convolve(np.nan_to_num(x), np.ones(k) / k, mode="same")


def onset_decomp(R, follow):
  """For each lead-braking onset, measure how many ms later each stage crosses a
  decel threshold. Robust to the cross-correlation shape/noise problems: it times
  real event onsets instead of correlating smoothed waveforms."""
  lead_a = smooth(R["vlead_deriv"], 0.15)        # lightly-cleaned raw radar lead accel
  REF, LVL = -0.7, -0.5                           # m/s^2: onset trigger, per-stage crossing
  cand = (lead_a < REF) & follow
  starts = np.where((~cand[:-1]) & cand[1:])[0] + 1
  sigs = {"aLeadK": R["aleadk"], "plan aTarget": R["atarget"], "cmd accel": R["cmd"], "vehicle aEgo": R["aego"]}
  win, quiet = int(2.5 / DT), int(1.0 / DT)
  rows: dict[str, list] = {k: [] for k in sigs}
  n, last_end = 0, -10 ** 9
  for s in starts:
    if s - last_end < quiet or s - quiet < 0:
      continue
    if np.nanmin(lead_a[s - quiet:s]) < REF:       # require lead was quiet (not already braking)
      continue
    end = min(s + win, len(R["t"]))
    ev, ok = {}, True
    for k, sig in sigs.items():
      idx = np.where(sig[s:end] < LVL)[0]
      if not len(idx):
        ok = False
        break
      ev[k] = idx[0] * DT * 1e3
    if ok:
      for k in sigs:
        rows[k].append(ev[k])
      n, last_end = n + 1, end
  return n, {k: (np.median(v) if v else np.nan) for k, v in rows.items()}


def best_lag(x, y, base_valid):
  valid = base_valid & np.isfinite(x) & np.isfinite(y)
  xd, yd = detrend(x), detrend(y)
  n = len(xd)
  best_l, best_c = 0, -2.0
  for l in range(int(MAX_LAG_S / DT)):
    m = valid[:n - l] & valid[l:] if l else valid
    if m.sum() < 100:
      continue
    a, b = (xd[:n - l][m], yd[l:][m]) if l else (xd[m], yd[m])
    if np.std(a) < 1e-3 or np.std(b) < 1e-3:
      continue
    c = float(np.corrcoef(a, b)[0, 1])
    if c > best_c:
      best_c, best_l = c, l
  return best_l * DT * 1e3, best_c


def main(route):
  print(f"reading {route} ...")
  cont, plan_src = collect(route)
  R = resample(cont)

  follow_lead = (np.isfinite(R["vego"]) & (R["vego"] > FOLLOW_MIN_VEGO) &
                 np.isfinite(R["has_lead"]) & (R["has_lead"] > 0.5))
  # only measure openpilot's own reaction: longitudinal engaged, no pedal override
  engaged = (np.isfinite(R["longactive"]) & (R["longactive"] > 0.5) &
             (R["brake"] < 0.5) & (R["gas"] < 0.5))
  follow = follow_lead & engaged
  dur = len(R["t"]) * DT
  print(f"\n{dur:.0f}s of log, {follow.mean() * 100:.0f}% openpilot-engaged lead-following above {FOLLOW_MIN_VEGO:.0f} m/s")
  hard = np.isfinite(R["aego"]) & (R["aego"] < DECEL_EVENT) & follow_lead
  if hard.sum():
    op = (hard & engaged).sum() / hard.sum()
    print(f"hard braking (<{DECEL_EVENT} m/s^2) while following: {hard.sum() * DT:.0f}s total, "
          f"{op * 100:.0f}% openpilot-engaged, {(1 - op) * 100:.0f}% manual/override")
  if plan_src:
    tot = len(plan_src)
    dist = ", ".join(f"{PLAN_SOURCE.get(s, s)} {100 * plan_src.count(s) / tot:.0f}%"
                     for s in sorted(set(plan_src)))
    print(f"longitudinal plan source: {dist}")
  if follow.sum() < 200:
    raise SystemExit("not enough lead-following in this route to estimate lags")

  # Focus on braking events: cross-correlation needs real deceleration dynamics,
  # not steady cruise, or the lags come back uncorrelated/unreliable.
  aego = R["aego"]
  decel = np.isfinite(aego) & (aego < DECEL_EVENT) & follow
  n_events = int(np.sum(np.diff(decel.astype(int)) == 1))
  pad = int(EVENT_PAD / DT)
  event_mask = (np.convolve(decel.astype(float), np.ones(2 * pad + 1), mode="same") > 0) & follow
  print(f"braking events (aEgo < {DECEL_EVENT} m/s^2): {n_events}, "
        f"min aEgo {np.nanmin(aego):.1f} m/s^2, event windows total {event_mask.sum() * DT:.0f}s")
  mask = event_mask if event_mask.sum() > 500 else follow
  print(f"lag basis: {'braking-event windows' if mask is event_mask else 'ALL following (no strong events here - look at other segments)'}")

  stages = [
    ("model lead a  -> vLead deriv ", "model_a",     "vlead_deriv", "model anticipation vs radar kinematics"),
    ("vLead deriv   -> aLeadK      ", "vlead_deriv", "aleadk",      "radar accel Kalman smoothing"),
    ("aLeadK        -> plan aTarget", "aleadk",      "atarget",     "MPC planning"),
    ("plan aTarget  -> cmd accel   ", "atarget",     "cmd",         "long control"),
    ("cmd accel     -> vehicle aEgo", "cmd",         "aego",        "actuation + powertrain"),
  ]
  print("\nper-stage lag (deceleration dynamics, lead-following only):")
  print(f"  {'stage':<30} {'lag':>8}  {'corr':>5}  what it is")
  results = []
  for label, xk, yk, desc in stages:
    lag, corr = best_lag(R[xk], R[yk], mask)
    flag = "" if corr >= MIN_CORR else "  (low corr - unreliable)"
    print(f"  {label:<30} {lag:>6.0f}ms  {corr:>5.2f}  {desc}{flag}")
    if corr >= MIN_CORR:
      results.append((lag, label.strip(), desc))

  for label, xk, yk in (("model lead a -> cmd accel", "model_a", "cmd"),
                        ("aLeadK       -> vehicle aEgo", "aleadk", "aego")):
    lag, corr = best_lag(R[xk], R[yk], mask)
    print(f"  TOTAL {label:<28} {lag:>6.0f}ms  {corr:>5.2f}")

  n_ev, onset = onset_decomp(R, follow)
  print(f"\nevent-onset latency from lead decel start ({n_ev} clean braking events):")
  prev = 0.0
  for k, v in onset.items():
    inc = (v - prev) if (np.isfinite(v) and np.isfinite(prev)) else float("nan")
    tag = {"aLeadK": "radar Kalman estimate", "plan aTarget": "MPC (jerk/cost smoothing)",
           "cmd accel": "long control", "vehicle aEgo": "actuation"}[k]
    print(f"  lead decel -> {k:<13} {v:>6.0f}ms  (+{inc:>4.0f}ms: {tag})")
    prev = v if np.isfinite(v) else prev

  if results:
    lag, label, desc = max(results)
    print(f"\nbinding stage: {label}  (~{lag:.0f}ms, {desc})")
    print("=> spend effort here first; tuning upstream of the binding stage won't be felt.")


if __name__ == "__main__":
  if len(sys.argv) < 2:
    print(__doc__)
    raise SystemExit("provide a route/segment/rlog identifier")
  main(sys.argv[1])
