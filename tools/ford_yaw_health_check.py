#!/usr/bin/env python3
"""Ford RCM yaw-sensor health check.

Some Ford RCMs broadcast implausible yaw rate (e.g. sign-inverted vs the IMU and the
steering geometry) while the signal's quality flag still reads OK. Since the stock
safety firmware and control code measure curvature from this yaw signal, a bad sensor
shows up as "steering limit exceeded" alerts at 20-35 mph and weak curve tracking.

This tool answers "is MY yaw sensor healthy?" from any drive's logs:
  - yaw_geo:  yaw rate implied by the steering angle via the vehicle model (reference)
  - yaw_can:  the RCM yaw rate the car broadcasts (what stock openpilot trusts)
  - yaw_imu:  the comma device gyro (independent cross-check)

A healthy sensor tracks the steering geometry in BOTH correlation AND gain:
corr(yaw_can, yaw_geo) > +0.9 with a median ratio near +1.0. The failure mode measured
on a faulty 2021 Explorer RCM was unstable gain -- across five months the ratio wandered
+0.42 / +0.45 / +0.62 / +0.71 / +0.92 / +1.80 / +2.11 (including two drives on the SAME
day at +2.11 and +0.45), with high positive correlation throughout -- which corrupts the
curvature measurement just as badly as a sign flip (a 2x gain error at curvature 0.005
is 2.5x the safety check's whole error band). A negative correlation (sign inversion)
is also broken. Because the gain wanders BETWEEN drives and occasionally passes through
+1.0 (a known-broken sensor measured +0.92 on one drive), ALWAYS run this on two or
three different routes: a ratio that moves is itself the fault signature.

Usage:
  tools/ford_yaw_health_check.py '<dongleid>|<route>'
  (works with rlogs; for qlog fallback append the LogReader qlog selector to the route)
"""
import math
import sys

import numpy as np

from openpilot.tools.lib.logreader import LogReader

MIN_SPEED = 5.0          # m/s; below this, geometry and yaw are both noise
SIGN_THRESH = 0.01       # rad/s; only count sign agreement when clearly turning
HEALTHY_CORR = 0.9
BROKEN_CORR = 0.5
# gain (median yaw_can / yaw_geo): healthy sits within vehicle-model tolerance of +1.0;
# bounds set from the faulty-RCM measurements in the module docstring
HEALTHY_RATIO = (0.85, 1.2)
BROKEN_RATIO = (0.65, 1.5)


def run(identifier):
  CP = None
  angle_offset_deg = 0.0
  rows = []               # v, steering_angle_deg, yaw_can, yaw_imu
  yaw_imu = float('nan')

  for m in LogReader(identifier):
    w = m.which()
    if w == 'carParams' and CP is None:
      CP = m.carParams
    elif w == 'liveParameters':
      angle_offset_deg = m.liveParameters.angleOffsetAverageDeg
    elif w == 'livePose':
      yaw_imu = m.livePose.angularVelocityDevice.z
    elif w == 'carState':
      cs = m.carState
      rows.append((cs.vEgo, cs.steeringAngleDeg - angle_offset_deg, cs.yawRate, yaw_imu))

  if CP is None or not rows:
    print('no carParams/carState in logs -- is this a full route identifier?')
    return 2
  if CP.brand != 'ford':
    print(f'not a Ford route (brand={CP.brand})')
    return 2

  from opendbc.car.vehicle_model import VehicleModel
  VM = VehicleModel(CP)

  a = np.array(rows, dtype=float)
  v, sa, yaw_can, imu = a.T
  moving = v > MIN_SPEED
  if moving.sum() < 500:
    print(f'only {moving.sum()} moving samples -- drive longer for a reliable verdict')
    return 2

  yaw_geo = np.array([VM.calc_curvature(math.radians(s), vv, 0.0) * vv for s, vv in zip(sa, v, strict=True)])

  def corr(x, y, mask):
    mask = mask & np.isfinite(x) & np.isfinite(y)
    if mask.sum() < 100 or np.std(x[mask]) < 1e-9 or np.std(y[mask]) < 1e-9:
      return float('nan')
    return float(np.corrcoef(x[mask], y[mask])[0, 1])

  c_can = corr(yaw_can, yaw_geo, moving)
  c_imu_raw = corr(imu, yaw_geo, moving)
  # the device gyro's z sign depends on mount orientation; orient it to the geometry
  imu_oriented = imu * (1.0 if (np.isnan(c_imu_raw) or c_imu_raw >= 0) else -1.0)
  c_imu = corr(imu_oriented, yaw_geo, moving)

  turning = moving & (np.abs(yaw_geo) > SIGN_THRESH)
  sign_disagree = float(np.mean(np.sign(yaw_can[turning]) != np.sign(yaw_geo[turning]))) if turning.sum() > 100 else float('nan')
  with np.errstate(divide='ignore', invalid='ignore'):
    ratio = yaw_can[turning] / yaw_geo[turning]
  median_ratio = float(np.median(ratio)) if turning.sum() > 100 else float('nan')

  print(f'route: {identifier}')
  print(f'platform: {CP.carFingerprint}   moving samples: {int(moving.sum())}   turning samples: {int(turning.sum())}')
  print(f'corr(RCM yaw, steering geometry):  {c_can:+.3f}   (healthy > {HEALTHY_CORR:+.1f})')
  print(f'corr(IMU yaw, steering geometry):  {c_imu:+.3f}   (sanity reference; should be > +0.9)')
  print(f'sign disagreement while turning:   {100.0 * sign_disagree:.1f}%')
  print(f'median yaw_can / yaw_geo ratio:    {median_ratio:+.2f}   (healthy ~ +1.0)')

  if not np.isnan(c_imu) and c_imu < HEALTHY_CORR:
    print('\nVERDICT: INCONCLUSIVE -- the IMU itself disagrees with steering geometry;')
    print('check device mount/calibration before trusting this run.')
    return 2
  if c_can < BROKEN_CORR:
    print('\nVERDICT: BROKEN -- your RCM yaw output does not track the steering geometry')
    print('(inverted or uncorrelated). Enable the "Use Pinion Yaw Sensor" toggle;')
    print('expect "Service AdvanceTrac"-style symptoms to correlate.')
    return 1
  if not np.isnan(median_ratio) and not (BROKEN_RATIO[0] <= median_ratio <= BROKEN_RATIO[1]):
    print('\nVERDICT: BROKEN -- your RCM yaw tracks the steering geometry in shape but at')
    print(f'the wrong gain ({median_ratio:+.2f}x instead of ~+1.0x). This is the measured')
    print('faulty-Explorer-RCM signature (gain wandered between +0.45x and +2.11x across')
    print('same-day drives) and corrupts the curvature measurement as badly as a sign flip.')
    print('Enable the "Use Pinion Yaw Sensor" toggle.')
    return 1
  if c_can >= HEALTHY_CORR and HEALTHY_RATIO[0] <= median_ratio <= HEALTHY_RATIO[1]:
    print('\nVERDICT: HEALTHY -- your yaw sensor tracks the steering geometry in shape and')
    print('gain. You do not need the "Use Pinion Yaw Sensor" toggle. (Consider')
    print('re-running on one or two more drives: a gain that moves between drives is the')
    print('fault signature even when a single drive looks acceptable.)')
    return 0
  print('\nVERDICT: MARGINAL -- correlation or gain is off but not conclusively broken;')
  print('re-run on a longer, curvier drive (and compare the gain across drives) before deciding.')
  return 2


if __name__ == '__main__':
  if len(sys.argv) != 2:
    print(__doc__)
    sys.exit(2)
  sys.exit(run(sys.argv[1]))
