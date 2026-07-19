#!/usr/bin/env python3
"""Replay validation of the pinion-sourced curvature measurement against real rlogs.

Simulates the STEER_ANGLE_CURVATURE firmware semantics per lateral frame:
  - angle_meas: 6-sample buffer of pinion-derived curvature CAN units (Explorer
    geometry row of the ford.h table, QF-gated)
  - steer_angle_cmd_checks error section (curvature mode, gate 10 m/s, band 0.003):
    outside [meas±err] AND not converging -> violation
  - ford_shadow_curvature_error_check (angle mode, same gate/band, no converge term)

Also compares the Python-layer deviation-clip bite rate between the vehicle-model
(pinion) measurement and the yaw-rate measurement.

Usage:
  FORD_REPLAY_DONGLE_ID=<dongleid> tools/ford_pinion_replay.py <route>[:<seg-slice>] ...
  e.g. tools/ford_pinion_replay.py 00000001--255eabb4d0 000000d9--89f2063192:0:61:2
"""
import os
import sys

import numpy as np
from opendbc.can.parser import CANParser
from openpilot.tools.lib.logreader import LogReader

DONGLE = os.environ.get('FORD_REPLAY_DONGLE_ID', '')  # set to your comma dongle ID
DEG_TO_CAN = 50000
MAX_ANGLE_ERR = 150       # 0.003, the pinion-path band
GATE = 10.0               # m/s
SLIP, SR, WB = -0.00055447339, 16.8, 3.025  # ford.h geometry table, FORD_EXPLORER_MK6 row
CURVATURE_ERROR = 0.002   # python layer band


def pinion_curv(angle_deg, speed):
  s = np.maximum(speed, 0.1)
  cf = 1. / (1. - (SLIP * s * s)) / WB
  return np.radians(angle_deg) * cf / SR


def run_route(route, segs):
  rows = []
  for seg in segs:
    try:
      lr = LogReader(f'{DONGLE}|{route}/{seg}')
    except Exception:
      continue
    cp_tx = CANParser('ford_lincoln_base_pt', [('LateralMotionControl', 0)], 0)
    cp_rx = CANParser('ford_lincoln_base_pt', [('SteeringPinion_Data', 0)], 0)
    last = {'pin': None, 'qf': 0, 'v': 0.0, 'eng': False, 'sp': False,
            'des': 0.0, 'yaw': 0.0, 'sa': 0.0}
    for m in lr:
      w = m.which()
      if w == 'can':
        frames = [(c.address, c.dat, c.src) for c in m.can]
        if cp_rx.update([(m.logMonoTime, frames)]):
          last['pin'] = cp_rx.vl['SteeringPinion_Data']['StePinComp_An_Est']
          last['qf'] = int(cp_rx.vl['SteeringPinion_Data']['StePinCompAnEst_D_Qf'])
      elif w == 'carState':
        last['v'] = m.carState.vEgo
        last['sp'] = m.carState.steeringPressed
        last['yaw'] = m.carState.yawRate
        last['sa'] = m.carState.steeringAngleDeg
      elif w == 'selfdriveStateSP':
        last['eng'] = m.selfdriveStateSP.mads.active
      elif w == 'carControl':
        last['des'] = m.carControl.actuators.curvature
      elif w == 'sendcan':
        frames = [(c.address, c.dat, c.src) for c in m.sendcan]
        if cp_tx.update([(m.logMonoTime, frames)]) and last['pin'] is not None:
          vl = cp_tx.vl['LateralMotionControl']
          rows.append((last['v'], last['pin'], last['qf'],
                       vl['LatCtlCurv_No_Actl'], vl['LatCtlPath_An_Actl'],
                       last['eng'], last['sp'], last['des'], last['yaw'], last['sa']))
  if not rows:
    return None
  a = np.array(rows, dtype=float)
  v, pin, qf, cmd, pa, eng, sp, des, yaw, sa = (a[:, i] for i in range(10))
  eng = eng.astype(bool)
  sp = sp.astype(bool)

  # firmware measurement: pinion -> curvature CAN units (sign: pinion correlates + with
  # wire cmd; firmware keeps native sign)
  k_pin = pinion_curv(pin, v)
  # empirical sign alignment to wire cmd (like firmware: both Ford-native convention)
  al = eng & (np.abs(cmd) > 0.001)
  s = 1.0
  if al.sum() > 200:
    s = np.sign(np.corrcoef(k_pin[al], cmd[al])[0, 1])
  meas_can = s * k_pin * DEG_TO_CAN

  # 6-frame rolling min/max of measurement (angle_meas buffer @ ~100 Hz vs 20 Hz cmd --
  # conservative: use per-cmd-frame value, window 6 cmd frames = wider window)
  n = len(meas_can)
  mn = np.empty(n)
  mx = np.empty(n)
  for i in range(n):
    lo = max(0, i - 5)
    mn[i] = meas_can[lo:i+1].min()
    mx[i] = meas_can[lo:i+1].max()

  cmd_can = cmd * DEG_TO_CAN
  dcmd = np.diff(cmd_can, prepend=cmd_can[0])
  hands_off = eng & ~sp & (v > GATE) & (qf == 3)

  # curvature mode check (with converge semantics)
  above = cmd_can > (mx + MAX_ANGLE_ERR + 1)
  below = cmd_can < (mn - MAX_ANGLE_ERR - 1)
  viol_curv = (above & (dcmd >= 0)) | (below & (dcmd <= 0))
  m_curv = hands_off & (np.abs(cmd) > 1e-5)   # curvature actually commanded
  # shadow check (angle mode: cmd==0; shadow = desired kappa, no converge term)
  shadow_can = -des * DEG_TO_CAN * s * np.sign(np.corrcoef(-des[al], cmd[al])[0, 1]) if al.sum() > 200 else -des * DEG_TO_CAN
  viol_shadow = (shadow_can > (mx + MAX_ANGLE_ERR + 1)) | (shadow_can < (mn - MAX_ANGLE_ERR - 1))
  m_shadow = hands_off & (np.abs(cmd) <= 1e-5) & (np.abs(pa) > 1e-4)  # angle mode active frames

  out = {}
  out['frames'] = n
  out['curv_mode_frames'] = int(m_curv.sum())
  out['curv_fault_pct'] = round(100 * float(viol_curv[m_curv].mean()), 3) if m_curv.sum() else None
  out['angle_mode_frames'] = int(m_shadow.sum())
  out['shadow_fault_pct'] = round(100 * float(viol_shadow[m_shadow].mean()), 3) if m_shadow.sum() else None
  # speed split for curvature mode (the historical 20-35mph zone)
  for lo, hi, lbl in [(10, 15.6, '22-35mph'), (15.6, 30, '>35mph')]:
    mm = m_curv & (v >= lo) & (v < hi)
    out[f'curv_fault_{lbl}'] = round(100 * float(viol_curv[mm].mean()), 3) if mm.sum() > 100 else f'n={mm.sum()}'
    ms = m_shadow & (v >= lo) & (v < hi)
    out[f'shadow_fault_{lbl}'] = round(100 * float(viol_shadow[ms].mean()), 3) if ms.sum() > 100 else f'n={ms.sum()}'

  # Python layer: clip-bite with the VM measurement (no offset -- conservative) vs yaw
  k_vm = -np.radians(sa) / (SR * WB) / np.maximum(1 - SLIP * v * v, 0.3)
  k_yaw = -yaw / np.maximum(v, 0.1)
  mm = hands_off & (np.abs(des) > 0.0015) & (v > 9)
  for name, k in [('vm', k_vm), ('yaw', k_yaw)]:
    clipped = (des > k + CURVATURE_ERROR) | (des < k - CURVATURE_ERROR)
    out[f'py_clip_bite_{name}_pct'] = round(100 * float(clipped[mm].mean()), 1) if mm.sum() else None
  return out


def _parse_route_arg(arg):
  parts = arg.split(':')
  segs = range(100) if len(parts) == 1 else range(*[int(x) for x in parts[1:]])
  return parts[0], segs


if __name__ == '__main__':
  if len(sys.argv) < 2 or not DONGLE:
    print(__doc__)
    sys.exit(2)
  for route_arg in sys.argv[1:]:
    route, segs = _parse_route_arg(route_arg)
    print(f'\n===== {route} =====')
    r = run_route(route, segs)
    if r is None:
      print('  no data loaded (are the rlogs uploaded?)')
      continue
    for key, val in r.items():
      print(f'  {key}: {val}')
