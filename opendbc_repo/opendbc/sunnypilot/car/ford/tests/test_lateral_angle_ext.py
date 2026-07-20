"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""

# Unit tests for angle-mode shadow-curvature publishing (bp_kappa_cmd).
#
# The shadow value is consumed by carcontroller as the input to ford.h's angle-mode
# deviation check (Lane_Assist_Data1 bytes 5-6, judged against angle_meas). These tests
# pin the truthfulness contract: whenever the planner kappa cannot honestly describe the
# car's steering -- inactive, human-turn override, stall blip, driver pressing -- the
# published shadow must equal the measured curvature, so the panda-latched value always
# stays inside the check's band and re-engage frames never compare a stale zero against
# real measured curvature.

import math
import unittest
from dataclasses import dataclass
from unittest import mock

from opendbc.car import structs
from opendbc.car.ford.values import CarControllerParams
from opendbc.car.interfaces import scale_tire_stiffness
from opendbc.sunnypilot.car.ford import lateral_curv_ext
from opendbc.sunnypilot.car.ford.values_ext import FordSafetyFlagsSP
from opendbc.sunnypilot.car.ford.lateral_curv_ext import LateralCurvExt
from opendbc.sunnypilot.car.ford.lateral_angle_ext import LateralAngleExt


def _explorer_cp():
  CP = structs.CarParams()
  CP.carFingerprint = 'FORD_EXPLORER_MK6'
  CP.mass = 2050.
  CP.wheelbase = 3.025
  CP.steerRatio = 16.8
  CP.centerToFront = CP.wheelbase * 0.44
  CP.tireStiffnessFactor = 0.82
  CP.tireStiffnessFront, CP.tireStiffnessRear = scale_tire_stiffness(
    CP.mass, CP.wheelbase, CP.centerToFront, CP.tireStiffnessFactor)
  return CP


class _FakeLiveDelay:
  lateralDelay = 0.2


class _FakeSubMaster:
  def __init__(self, *args, **kwargs):
    self.updated = {s: False for s in ('modelV2', 'liveParameters', 'selfdriveState', 'radarState', 'liveDelay')}

  def update(self, timeout=0):
    pass

  def __getitem__(self, key):
    if key == 'liveDelay':
      return _FakeLiveDelay()
    raise KeyError(key)


class _ForcedDetector:
  def __init__(self, active):
    self.active = active

  def update(self, *_args):
    return self.active

  def reset(self):
    pass


@dataclass
class _CSOut:
  vEgoRaw: float = 15.0
  vEgo: float = 15.0
  steeringPressed: bool = False
  steeringAngleDeg: float = 0.0
  yawRate: float = 0.0


class _CS:
  def __init__(self, **kwargs):
    self.out = _CSOut(**kwargs)
    self.lat_ctl_lim_stat = 0
    self.la_act_avail = -1  # no PSCM availability broadcast unless a test sets one


@dataclass
class _CC:
  latActive: bool = True


@dataclass
class _Actuators:
  curvature: float = 0.0


class _Harness(LateralCurvExt, LateralAngleExt):
  """Mirrors CarController's mixin composition (see carcontroller.py)."""

  def __init__(self, CP, CP_SP=None):
    with mock.patch.object(lateral_curv_ext.messaging, 'SubMaster', _FakeSubMaster):
      LateralCurvExt.__init__(self, CP, CP_SP)
    LateralAngleExt.__init__(self, CP, CP_SP)


def _pinion_harness(flag):
  """Harness with the STEER_ANGLE_CURVATURE flag set (or not) on CP_SP, detector stubbed."""
  CP = _explorer_cp()
  CP_SP = structs.CarParamsSP()
  if flag:
    CP_SP.safetyParam |= FordSafetyFlagsSP.STEER_ANGLE_CURVATURE
  ext = _Harness(CP, CP_SP)
  ext.human_turn_detector = _ForcedDetector(False)
  return ext, CP


class TestShadowCurvaturePublishing(unittest.TestCase):
  V_EGO = 15.0
  YAW_RATE = 0.75  # rad/s -> measured curvature = -0.75 / 15 = -0.05 (OP convention)

  def setUp(self):
    self.CP = _explorer_cp()
    self.ext = _Harness(self.CP)
    self.ext.human_turn_detector = _ForcedDetector(False)
    self.cs = _CS(vEgoRaw=self.V_EGO, vEgo=self.V_EGO, yawRate=self.YAW_RATE)
    self.measured = -self.YAW_RATE / self.V_EGO

  def _update(self, lat_active=True):
    return self.ext.update_angle_strategy(_CC(latActive=lat_active), self.cs, _Actuators(curvature=0.01), self.CP)

  def test_inactive_publishes_measured(self):
    result = self._update(lat_active=False)
    self.assertEqual(result.path_angle, 0.0)
    self.assertAlmostEqual(self.ext.bp_kappa_cmd, self.measured)

  def test_human_turn_override_publishes_measured(self):
    self.ext.human_turn_detector = _ForcedDetector(True)
    result = self._update()
    self.assertTrue(self.ext.angle_human_turn_active)
    self.assertEqual(result.path_angle, 0.0)
    self.assertAlmostEqual(self.ext.bp_kappa_cmd, self.measured)

  def test_stall_blip_publishes_measured(self):
    self.ext.stall_blip_frames_left = 3
    result = self._update()
    self.assertTrue(self.ext.angle_stall_blip_active)
    self.assertEqual(result.path_angle, 0.0)
    self.assertAlmostEqual(self.ext.bp_kappa_cmd, self.measured)

  def test_pressed_publishes_measured(self):
    self.cs.out.steeringPressed = True
    self._update()
    self.assertFalse(self.ext.angle_human_turn_active)
    self.assertAlmostEqual(self.ext.bp_kappa_cmd, self.measured)

  def test_hands_off_publishes_clipped_planner_kappa(self):
    # planner wants +0.01 while measured is -0.05: the deviation clip (active above 9 m/s)
    # bounds the shadow to measured + CURVATURE_ERROR, not measured itself -- hands-off
    # behavior is unchanged by the truthful-shadow sites.
    self._update()
    expected = self.measured + CarControllerParams.CURVATURE_ERROR
    self.assertAlmostEqual(self.ext.bp_kappa_cmd, expected)
    self.assertNotAlmostEqual(self.ext.bp_kappa_cmd, self.measured)
    self.assertTrue(self.ext.bp_curvature_deviation_limited)


class TestMeasurementSelection(unittest.TestCase):
  """get_current_curvature must select by the CP_SP STEER_ANGLE_CURVATURE flag: yaw rate
  by default (stock ford.h angle_meas family), pinion angle via the vehicle model when
  the steering-angle curvature measurement is enabled (pinion ford.h angle_meas family).
  """

  V_EGO = 15.0

  def test_default_is_yaw_rate(self):
    ext, _ = _pinion_harness(flag=False)
    cs = _CS(vEgoRaw=self.V_EGO, yawRate=0.75, steeringAngleDeg=30.0)
    self.assertFalse(ext.bp_pinion_curvature_enabled)
    self.assertAlmostEqual(ext.get_current_curvature(cs), -0.75 / self.V_EGO)

  def test_flag_selects_pinion_vehicle_model(self):
    from opendbc.car.vehicle_model import VehicleModel
    ext, CP = _pinion_harness(flag=True)
    cs = _CS(vEgoRaw=self.V_EGO, yawRate=0.75, steeringAngleDeg=30.0)
    self.assertTrue(ext.bp_pinion_curvature_enabled)
    expected = -VehicleModel(CP).calc_curvature(math.radians(30.0), self.V_EGO, 0.0)
    self.assertAlmostEqual(ext.get_current_curvature(cs), expected)
    self.assertNotAlmostEqual(ext.get_current_curvature(cs), -0.75 / self.V_EGO)


class TestInitializeFord(unittest.TestCase):
  def test_safety_param_stays_a_plain_int(self):
    """card serializes CP_SP to capnp, which rejects enum subclasses of int -- an
    IntFlag-typed safetyParam crashed card on-device. Pin the exact type."""
    from opendbc.sunnypilot.car.interfaces import _initialize_ford
    CP = structs.CarParams()
    CP.brand = 'ford'
    CP.carFingerprint = 'FORD_EXPLORER_MK6'
    CP_SP = structs.CarParamsSP()
    _initialize_ford(CP, CP_SP, {"FordPrefSteerAngleCurvature": True})
    self.assertEqual(CP_SP.safetyParam, 0xb)  # flag | (explorer index 5 << 1)
    self.assertIs(type(CP_SP.safetyParam), int)


class TestLowSpeedStallRescue(unittest.TestCase):
  """With the pinion measurement enabled, stall detection extends below the deviation
  clip's 9 m/s gate (down to 5 m/s, the ford.h path-offset floor), charging on the raw
  gap where the clip can never bind. With the flag off, gating is bit-identical to
  before: nothing charges below 9 m/s (yaw measurement distrust stands)."""

  def _drive_stalled(self, ext, CP, v_ego, frames):
    # hands-off, measured curvature 0 (wheel straight), planner asking 0.01 -> raw gap
    # 0.01 > _STALL_GAP_MIN; below 9 m/s the deviation clip is inert so devLim stays False
    cs = _CS(vEgoRaw=v_ego, vEgo=v_ego, yawRate=0.0, steeringAngleDeg=0.0)
    for _ in range(frames):
      ext.update_angle_strategy(_CC(latActive=True), cs, _Actuators(curvature=0.01), CP)

  def test_pinion_rescues_below_clip_gate(self):
    ext, CP = _pinion_harness(flag=True)
    self._drive_stalled(ext, CP, v_ego=6.0, frames=12)
    self.assertEqual(ext.stall_blip_count, 1)  # blip fired from gap-only accumulation

  def test_pinion_inert_below_stall_gate(self):
    ext, CP = _pinion_harness(flag=True)
    self._drive_stalled(ext, CP, v_ego=4.5, frames=12)
    self.assertEqual(ext.stall_blip_count, 0)
    self.assertEqual(ext.stall_blip_hold_s, 0.0)

  def test_yaw_mode_unchanged_below_gate(self):
    ext, CP = _pinion_harness(flag=False)
    self._drive_stalled(ext, CP, v_ego=6.0, frames=12)
    self.assertEqual(ext.stall_blip_count, 0)
    self.assertEqual(ext.stall_blip_hold_s, 0.0)


class TestStallFractionalGate(unittest.TestCase):
  """The stall pulse must arm only on a fractional delivery failure (< 0.65x of the
  demand), never on an honest deep-curve entry transient (0.7-0.85x of a large demand)
  whose absolute gap clears _STALL_GAP_MIN on magnitude alone. On-road, such mid-curve
  pulses each triggered a driver grab within 0.2 s."""

  def _drive(self, desired, measured, frames=20, v_ego=10.0):
    # yaw mode (flag off): measured curvature comes exactly from yawRate / v; at v > 9
    # the deviation clip binds on the gap, so devLim provides the charging path.
    ext, CP = _pinion_harness(flag=False)
    cs = _CS(vEgoRaw=v_ego, vEgo=v_ego, yawRate=-measured * v_ego, steeringAngleDeg=0.0)
    for _ in range(frames):
      ext.update_angle_strategy(_CC(latActive=True), cs, _Actuators(curvature=desired), CP)
    return ext

  def test_entry_transient_never_fires(self):
    # delivering 0.72x of a deep demand: gap 0.007 > _STALL_GAP_MIN, but not a stall
    ext = self._drive(desired=0.025, measured=0.018)
    self.assertEqual(ext.stall_blip_count, 0)
    self.assertEqual(ext.stall_blip_hold_s, 0.0)

  def test_true_stall_still_fires(self):
    # delivering 0.2x of the same demand: fractional failure -> pulse
    ext = self._drive(desired=0.025, measured=0.005)
    self.assertEqual(ext.stall_blip_count, 1)


class TestStallPinionFloor(unittest.TestCase):
  """With the pinion measurement, the stall gap floor drops to 1.5x CURVATURE_ERROR:
  the deviation clip caps the wire command at measured + CURVATURE_ERROR, so partial
  attenuation pins the observable gap just under the 2.0x floor (on-road: p50 0.0032
  vs 0.0040). Yaw mode keeps the 2.0x floor bit-identically."""

  def _drive(self, flag, desired, measured, v_ego, frames=20):
    ext, CP = _pinion_harness(flag=flag)
    if flag:
      # pinion measurement: set the steering angle via the vehicle model's own inverse
      sa_deg = math.degrees(ext.VM.get_steer_from_curvature(-measured, v_ego, 0.0))
      cs = _CS(vEgoRaw=v_ego, vEgo=v_ego, yawRate=0.0, steeringAngleDeg=sa_deg)
    else:
      cs = _CS(vEgoRaw=v_ego, vEgo=v_ego, yawRate=-measured * v_ego, steeringAngleDeg=0.0)
    for _ in range(frames):
      ext.update_angle_strategy(_CC(latActive=True), cs, _Actuators(curvature=desired), CP)
    return ext

  def test_pinion_fires_between_floors(self):
    # gap 0.0038 (in (1.5x, 2.0x)), delivering 0.62x: the clip-hidden partial
    # attenuation the 2.0x floor structurally cannot see (v=6: raw-gap charging)
    ext = self._drive(flag=True, desired=0.010, measured=0.0062, v_ego=6.0)
    self.assertEqual(ext.stall_blip_count, 1)

  def test_yaw_floor_unchanged(self):
    # same gap in yaw mode at v=10 (clip binding supplies charging): below 2.0x -> inert
    ext = self._drive(flag=False, desired=0.010, measured=0.0062, v_ego=10.0)
    self.assertEqual(ext.stall_blip_count, 0)
    self.assertEqual(ext.stall_blip_hold_s, 0.0)

  def test_pinion_inert_below_its_floor(self):
    # gap 0.0029 < 1.5x with a real fractional deficit (0.64x): healthy-tracking margin
    ext = self._drive(flag=True, desired=0.008, measured=0.0051, v_ego=6.0)
    self.assertEqual(ext.stall_blip_count, 0)


class TestAvail0StallGate(unittest.TestCase):
  """While the PSCM positively broadcasts its availability-policy derate
  (LaActAvail_D_Actl == 0, via CS.la_act_avail), the delivery deficit is the policy and
  a mode-0 pulse cannot restore it -- the stall detector must only fire when a
  press/human-turn/engage interaction is recent enough for press-type attenuation to
  plausibly be stacked on top. No broadcast (la_act_avail = -1) and avail=2 keep
  today's behavior bit-identically."""

  def _stalled_cs(self, avail, v_ego=6.0):
    cs = _CS(vEgoRaw=v_ego, vEgo=v_ego, yawRate=0.0, steeringAngleDeg=0.0)
    cs.la_act_avail = avail
    return cs

  def _drive(self, ext, CP, cs, frames):
    for _ in range(frames):
      ext.update_angle_strategy(_CC(latActive=True), cs, _Actuators(curvature=0.01), CP)

  def _drive_aged(self, avail):
    # consume the engage-edge trigger, then age past the rescue window before the stall
    ext, CP = _pinion_harness(flag=True)
    cs = self._stalled_cs(avail)
    self._drive(ext, CP, cs, 1)
    ext.attn_trigger_age_s = 600.0
    self._drive(ext, CP, cs, 12)
    return ext

  def test_policy_stall_without_trigger_stays_quiet(self):
    ext = self._drive_aged(avail=0)
    self.assertEqual(ext.stall_blip_count, 0)
    self.assertEqual(ext.stall_blip_hold_s, 0.0)

  def test_no_broadcast_unchanged(self):
    ext = self._drive_aged(avail=-1)
    self.assertEqual(ext.stall_blip_count, 1)

  def test_avail1_also_suppresses(self):
    # LaActAvail is a feature matrix; value 1 (LCA/LKA suppressed, LDW available) is
    # policy-suppressed centering just like 0
    ext = self._drive_aged(avail=1)
    self.assertEqual(ext.stall_blip_count, 0)

  def test_avail2_unchanged(self):
    ext = self._drive_aged(avail=2)
    self.assertEqual(ext.stall_blip_count, 1)

  def test_engage_edge_reopens_rescue(self):
    # engagement sag at avail=0 is pulse-curable: the engage edge zeroes the trigger age
    ext, CP = _pinion_harness(flag=True)
    cs = self._stalled_cs(avail=0)
    self._drive(ext, CP, cs, 12)
    self.assertEqual(ext.stall_blip_count, 1)

  def test_press_release_reopens_rescue(self):
    ext, CP = _pinion_harness(flag=True)
    cs = self._stalled_cs(avail=0)
    self._drive(ext, CP, cs, 1)
    ext.attn_trigger_age_s = 600.0
    cs.out.steeringPressed = True
    self._drive(ext, CP, cs, 2)
    cs.out.steeringPressed = False
    self._drive(ext, CP, cs, 12)
    self.assertEqual(ext.stall_blip_count, 1)


class TestPandaMirrorFrame(unittest.TestCase):
  """Panda-facing quantities (published shadow, deviation-clip band) use the panda's
  measurement frame: raw pinion angle through the FIXED CP geometry -- no live angle
  offset, no roll compensation, no live steer ratio. Control keeps the compensated
  measurement (get_current_curvature)."""

  def _lp(self):
    from types import SimpleNamespace
    return SimpleNamespace(angleOffsetDeg=2.0, roll=0.05)

  def test_mirror_ignores_live_offset_and_roll(self):
    ext, _ = _pinion_harness(flag=True)
    cs = _CS(vEgoRaw=15.0, vEgo=15.0, steeringAngleDeg=30.0, yawRate=0.0)
    base = ext.get_panda_mirror_curvature(cs)
    ext.lp = self._lp()
    self.assertAlmostEqual(ext.get_panda_mirror_curvature(cs), base)   # panda frame unmoved
    self.assertNotAlmostEqual(ext.get_current_curvature(cs), base)     # control frame moved

  def test_pressed_shadow_publishes_mirror(self):
    ext, CP = _pinion_harness(flag=True)
    ext.lp = self._lp()
    cs = _CS(vEgoRaw=15.0, vEgo=15.0, steeringAngleDeg=30.0, yawRate=0.0, steeringPressed=True)
    ext.update_angle_strategy(_CC(latActive=True), cs, _Actuators(curvature=0.01), CP)
    self.assertAlmostEqual(ext.bp_kappa_cmd, ext.get_panda_mirror_curvature(cs))

  def test_yaw_mode_frames_identical(self):
    ext, _ = _pinion_harness(flag=False)
    cs = _CS(vEgoRaw=15.0, vEgo=15.0, yawRate=0.75, steeringAngleDeg=30.0)
    self.assertAlmostEqual(ext.get_panda_mirror_curvature(cs), ext.get_current_curvature(cs))


class TestEntryAgilityBandLead(unittest.TestCase):
  """Predictive deviation-clip band center: leads only while the car is confirmed moving
  toward the demand, keeps total wire deviation within CURVATURE_ERROR + _BAND_LEAD_MAX,
  collapses to the static band on a frozen plant (and always on yaw), and never changes
  what the stall detector's charging sees (static-band keyed). Profiles co-ramp the
  demand with the measurement (planner demand tracks the car on real entries); a step
  demand against a slow plant is the stall detector's territory, not this feature's."""

  V = 12.0
  ERR = CarControllerParams.CURVATURE_ERROR
  LEAD = 0.0005
  RAMP = [0.000625 * i for i in range(13)]  # briskly turning in, final 0.0075

  @staticmethod
  def _harness(flag):
    # planner-only demand: the empty modelV2 in the fake SubMaster would otherwise halve
    # the effective request through the predicted-curvature blend (b = 0.5)
    ext, CP = _pinion_harness(flag=flag)
    ext.path_angle_blend_ratio = 0.0
    return ext, CP

  def _cs_for_meas(self, ext, measured):
    sa_deg = math.degrees(ext.VM.get_steer_from_curvature(-measured, self.V, 0.0))
    return _CS(vEgoRaw=self.V, vEgo=self.V, steeringAngleDeg=sa_deg, yawRate=0.0)

  def _co_ramp(self, ext, CP, gap, meas_seq=None):
    # demand rides `gap` above a ramping measurement; gap in (ERR, stall floor) exercises
    # the clip without ever arming the stall detector
    for m in (meas_seq if meas_seq is not None else self.RAMP):
      ext.update_angle_strategy(_CC(latActive=True), self._cs_for_meas(ext, m),
                                _Actuators(curvature=m + gap), CP)

  def test_frozen_plant_static_band(self):
    ext, CP = self._harness(flag=True)
    for _ in range(10):
      ext.update_angle_strategy(_CC(latActive=True), self._cs_for_meas(ext, 0.004),
                                _Actuators(curvature=0.02), CP)
    self.assertEqual(ext.band_lead, 0.0)
    self.assertAlmostEqual(ext.bp_kappa_cmd, 0.004 + self.ERR, places=6)

  def test_following_plant_leads_and_clamps(self):
    ext, CP = self._harness(flag=True)
    self._co_ramp(ext, CP, gap=0.0025)
    final_meas = self.RAMP[-1]
    self.assertGreater(ext.bp_kappa_cmd, final_meas + self.ERR + 1e-9)     # led past static
    self.assertLessEqual(ext.bp_kappa_cmd, final_meas + self.ERR + self.LEAD + 1e-9)

  def test_sign_symmetry(self):
    ext, CP = self._harness(flag=True)
    self._co_ramp(ext, CP, gap=-0.0025, meas_seq=[-m for m in self.RAMP])
    final_meas = -self.RAMP[-1]
    self.assertLess(ext.bp_kappa_cmd, final_meas - self.ERR - 1e-9)
    self.assertGreaterEqual(ext.bp_kappa_cmd, final_meas - self.ERR - self.LEAD - 1e-9)

  def test_yaw_mode_never_leads(self):
    ext, CP = self._harness(flag=False)
    for m in self.RAMP:
      cs = _CS(vEgoRaw=self.V, vEgo=self.V, yawRate=-m * self.V, steeringAngleDeg=0.0)
      ext.update_angle_strategy(_CC(latActive=True), cs, _Actuators(curvature=m + 0.0025), CP)
    self.assertEqual(ext.band_lead, 0.0)
    self.assertAlmostEqual(ext.bp_kappa_cmd, self.RAMP[-1] + self.ERR, places=6)

  def test_lead_decays_when_following_stops(self):
    ext, CP = self._harness(flag=True)
    self._co_ramp(ext, CP, gap=0.0015)  # inside the static band: gate open, no binding
    self.assertGreater(ext.band_lead, 0.0)
    final = self.RAMP[-1]
    for _ in range(4):  # plant freezes; demand persists just outside the static band
      ext.update_angle_strategy(_CC(latActive=True), self._cs_for_meas(ext, final),
                                _Actuators(curvature=final + 0.0025), CP)
    self.assertEqual(ext.band_lead, 0.0)
    self.assertAlmostEqual(ext.bp_kappa_cmd, final + self.ERR, places=6)

  def test_stall_charging_sees_static_band(self):
    # demand just outside the static band but inside the led band: the actual clip is
    # unbound (no devlim), yet stall charging must still see the static-band truth
    ext, CP = self._harness(flag=True)
    self._co_ramp(ext, CP, gap=0.0025)
    m = self.RAMP[-1] + 0.000625  # ramp continues: follow-gate stays open
    ext.update_angle_strategy(_CC(latActive=True), self._cs_for_meas(ext, m),
                              _Actuators(curvature=m + self.ERR + 0.0003), CP)
    self.assertFalse(ext.bp_curvature_deviation_limited)
    self.assertTrue(ext.bp_stall_charge_bound)

  def test_road_speed_pinion_stall_still_fires(self):
    ext, CP = self._harness(flag=True)
    for _ in range(14):  # frozen plant, deep step demand: the detector's territory
      ext.update_angle_strategy(_CC(latActive=True), self._cs_for_meas(ext, 0.0),
                                _Actuators(curvature=0.01), CP)
    self.assertEqual(ext.stall_blip_count, 1)


class TestPressReleaseBlip(unittest.TestCase):
  # The hand-off blip must fire only on straight-ish roads (its design intent): a
  # mid-curve release must NOT trigger a 300 ms steering drop -- the press -> blip ->
  # lane-sag -> press cascade observed on-road. And the release must PERSIST for the
  # debounce window: a 50 ms grip fluctuation (route-27 100 Hz forensics) must neither
  # fire the pulse nor reset the press timer.

  RELEASE_FRAMES = 9  # > _PRESS_BLIP_DEBOUNCE_S (0.4 s) at 20 Hz

  def _drive(self, ext, CP, pressed, frames, curvature):
    cs = _CS(vEgoRaw=8.0, vEgo=8.0, yawRate=0.0, steeringAngleDeg=0.0, steeringPressed=pressed)
    for _ in range(frames):
      ext.update_angle_strategy(_CC(latActive=True), cs, _Actuators(curvature=curvature), CP)
    return ext

  def _press_then_release(self, curvature, release_frames=RELEASE_FRAMES):
    ext, CP = _pinion_harness(flag=True)
    self._drive(ext, CP, pressed=True, frames=15, curvature=curvature)  # > _PRESS_BLIP_MIN_S
    self._drive(ext, CP, pressed=False, frames=release_frames, curvature=curvature)
    return ext

  def test_no_blip_on_mid_curve_release(self):
    ext = self._press_then_release(curvature=0.008)  # 125 m curve
    self.assertEqual(ext.stall_blip_frames_left, 0)

  def test_blip_on_straight_release(self):
    ext = self._press_then_release(curvature=0.001)  # near-straight
    self.assertGreater(ext.stall_blip_frames_left, 0)

  def test_micro_release_does_not_fire(self):
    # 0.1 s release (2 frames) is inside the debounce window: no pulse
    ext = self._press_then_release(curvature=0.001, release_frames=2)
    self.assertEqual(ext.stall_blip_frames_left, 0)

  def test_micro_release_keeps_press_accumulating(self):
    # press, 0.1 s fluctuation, regrip: the press timer survives the gap, so the
    # eventual real hand-off still earns its pulse
    ext, CP = _pinion_harness(flag=True)
    self._drive(ext, CP, pressed=True, frames=15, curvature=0.001)
    self._drive(ext, CP, pressed=False, frames=2, curvature=0.001)   # micro-release
    self._drive(ext, CP, pressed=True, frames=2, curvature=0.001)    # regrip
    self.assertGreater(ext.press_timer_s, 0.5)  # not reset by the fluctuation
    self._drive(ext, CP, pressed=False, frames=self.RELEASE_FRAMES, curvature=0.001)
    self.assertGreater(ext.stall_blip_frames_left, 0)

  def test_curvature_gate_evaluated_at_fire_time(self):
    # straight at release, curved by the end of the debounce window -> no pulse
    ext, CP = _pinion_harness(flag=True)
    self._drive(ext, CP, pressed=True, frames=15, curvature=0.001)
    self._drive(ext, CP, pressed=False, frames=4, curvature=0.001)
    self._drive(ext, CP, pressed=False, frames=5, curvature=0.008)  # curve arrives mid-window
    self.assertEqual(ext.stall_blip_frames_left, 0)


class TestDeliveryCompensation(unittest.TestCase):
  """The measured-delivery compensation must scale ONLY the path_angle actuator signal,
  only when the pinion measurement is enabled, and stay inside every existing bound."""

  def _one_frame(self, flag, v_ego, desired=0.01):
    ext, CP = _pinion_harness(flag=flag)
    cs = _CS(vEgoRaw=v_ego, vEgo=v_ego, yawRate=0.0, steeringAngleDeg=0.0)
    result = ext.update_angle_strategy(_CC(latActive=True), cs, _Actuators(curvature=desired), CP)
    return ext, result

  def test_comp_applied_when_pinion_enabled(self):
    # v=10: comp is 1.30; measured 0 so the deviation clip pins kappa_cmd to +0.002 in
    # both cases -- the returned path_angle ratio is exactly the compensation factor
    from opendbc.sunnypilot.car.ford.lateral_angle_ext import _DELIVERY_COMP
    from numpy import interp
    _, r_off = self._one_frame(flag=False, v_ego=10.0)
    _, r_on = self._one_frame(flag=True, v_ego=10.0)
    bp, vv = _DELIVERY_COMP['FORD_EXPLORER_MK6']
    expected = float(interp(10.0, bp, vv))
    self.assertGreater(expected, 1.2)  # the band this exists for
    self.assertAlmostEqual(r_on.path_angle / r_off.path_angle, expected, places=5)

  def test_comp_inert_without_pinion_flag(self):
    # yaw-measured cars keep today's behavior bit-identically
    ext, r = self._one_frame(flag=False, v_ego=10.0)
    self.assertAlmostEqual(r.path_angle, 0.002 * 10.0 * ext.curvature_factor)

  def test_comp_no_op_at_crawl_speed(self):
    from opendbc.sunnypilot.car.ford.lateral_angle_ext import _DELIVERY_COMP
    from numpy import interp
    bp, vv = _DELIVERY_COMP['FORD_EXPLORER_MK6']
    self.assertEqual(float(interp(1.0, bp, vv)), 1.0)

  def test_comp_inert_on_unmeasured_platform(self):
    # the curve is a per-platform PSCM calibration: platforms without a measured row must
    # get NO compensation even with the pinion toggle on
    ext, CP = _pinion_harness(flag=True)
    CP.carFingerprint = 'FORD_MAVERICK_MK1'
    cs = _CS(vEgoRaw=10.0, vEgo=10.0, yawRate=0.0, steeringAngleDeg=0.0)
    r = ext.update_angle_strategy(_CC(latActive=True), cs, _Actuators(curvature=0.01), CP)
    self.assertAlmostEqual(r.path_angle, 0.002 * 10.0 * ext.curvature_factor)

  def test_clip_never_amplifies_the_drive(self):
    # measured curvature overshooting the request must not drag the steering command up
    # with it (the sticky-hold observed on-road): the shadow still follows measured for
    # the panda check, but the drive is capped at what control requested
    ext, CP = _pinion_harness(flag=True)
    v = 15.0
    cs = _CS(vEgoRaw=v, vEgo=v, yawRate=0.0, steeringAngleDeg=-32.75)  # measured ~ +0.01
    r = ext.update_angle_strategy(_CC(latActive=True), cs, _Actuators(curvature=0.004), CP)
    measured = ext.get_current_curvature(cs)
    self.assertGreater(measured, 0.009)
    # shadow: clipped toward measured (panda-honest, inside the band)
    self.assertAlmostEqual(ext.bp_kappa_cmd, measured - CarControllerParams.CURVATURE_ERROR, places=5)
    # drive: the blended request, NOT the clip-raised value
    from opendbc.sunnypilot.car.ford.lateral_angle_ext import _DELIVERY_COMP
    from numpy import interp
    request = 0.004 * (1.0 - ext.path_angle_blend_ratio)
    bp, vv = _DELIVERY_COMP['FORD_EXPLORER_MK6']
    expected_pa = request * v * ext.curvature_factor * float(interp(v, bp, vv))
    self.assertAlmostEqual(r.path_angle, expected_pa, places=5)
    self.assertLess(abs(r.path_angle), abs(ext.bp_kappa_cmd) * v * ext.curvature_factor)

  def test_comp_table_envelope(self):
    # guard future refits: monotonic breakpoints, factors within a SPEED-DEPENDENT
    # envelope -- the large low-speed boosts (PSCM derate territory, low lateral energy)
    # must never leak into mid/high-speed rows
    from opendbc.sunnypilot.car.ford.lateral_angle_ext import _DELIVERY_COMP
    for platform, (bp, vv) in _DELIVERY_COMP.items():
      self.assertEqual(bp, sorted(bp), platform)
      self.assertEqual(len(bp), len(vv), platform)
      for b, f in zip(bp, vv, strict=True):
        self.assertGreaterEqual(f, 0.9, platform)
        self.assertLessEqual(f, 2.25 if b < 9.0 else 1.5, f'{platform} @ {b} m/s')

  def test_comp_low_speed_region_applied(self):
    # the v6 low-speed extension must actually reach the actuator signal: at 4.5 m/s the
    # comp is 2.2 and the STEADY-STATE on/off path_angle ratio equals it exactly (the
    # clip is inert below the 9 m/s gate; the first frame from rest is soft-ROC-limited
    # by design, so drive several frames to convergence before comparing)
    from opendbc.sunnypilot.car.ford.lateral_angle_ext import _DELIVERY_COMP
    from numpy import interp

    def steady(flag):
      ext, CP = _pinion_harness(flag=flag)
      cs = _CS(vEgoRaw=4.5, vEgo=4.5, yawRate=0.0, steeringAngleDeg=0.0)
      for _ in range(10):
        r = ext.update_angle_strategy(_CC(latActive=True), cs, _Actuators(curvature=0.01), CP)
      return r.path_angle

    bp, vv = _DELIVERY_COMP['FORD_EXPLORER_MK6']
    expected = float(interp(4.5, bp, vv))
    self.assertGreaterEqual(expected, 2.0)
    self.assertAlmostEqual(steady(True) / steady(False), expected, places=5)

  def test_dbc_limit_holds_at_strongest_boost(self):
    # max curvature demand at the STRONGEST comp band (4.5-6 m/s, 2.2x), driven to
    # steady state: DBC value clip must still bound every frame
    from opendbc.sunnypilot.car.ford.lateral_angle_ext import FORD_DBC_PATH_ANGLE_MAX
    ext, CP = _pinion_harness(flag=True)
    cs = _CS(vEgoRaw=6.0, vEgo=6.0, yawRate=0.0, steeringAngleDeg=0.0)
    for _ in range(60):
      r = ext.update_angle_strategy(_CC(latActive=True), cs, _Actuators(curvature=0.02), CP)
      self.assertLessEqual(abs(r.path_angle), FORD_DBC_PATH_ANGLE_MAX)

  def test_comp_does_not_touch_shadow(self):
    # the shadow (bp_kappa_cmd) is judged by ford.h against measured curvature -- the
    # compensation must never inflate it
    ext_on, _ = self._one_frame(flag=True, v_ego=10.0)
    ext_off, _ = self._one_frame(flag=False, v_ego=10.0)
    self.assertAlmostEqual(ext_on.bp_kappa_cmd, 0.002)   # clipped to measured(0) + CURVATURE_ERROR
    self.assertAlmostEqual(ext_on.bp_kappa_cmd, ext_off.bp_kappa_cmd)

  def test_dbc_limit_holds_at_worst_case(self):
    # max curvature demand at the strongest comp band, driven to steady state: the DBC
    # value clip must still bound the output every frame
    from opendbc.sunnypilot.car.ford.lateral_angle_ext import FORD_DBC_PATH_ANGLE_MAX
    ext, CP = _pinion_harness(flag=True)
    cs = _CS(vEgoRaw=13.0, vEgo=13.0, yawRate=0.0, steeringAngleDeg=0.0)
    for _ in range(60):
      r = ext.update_angle_strategy(_CC(latActive=True), cs, _Actuators(curvature=0.02), CP)
      self.assertLessEqual(abs(r.path_angle), FORD_DBC_PATH_ANGLE_MAX)

  def test_soft_roc_still_limits_first_frame(self):
    # the rate limit applies AFTER compensation: the first frame from rest can never step
    # further than the soft ROC allows, no matter the boost
    ext, CP = _pinion_harness(flag=True)
    cs = _CS(vEgoRaw=13.0, vEgo=13.0, yawRate=0.0, steeringAngleDeg=0.0)
    r = ext.update_angle_strategy(_CC(latActive=True), cs, _Actuators(curvature=0.02), CP)
    from numpy import interp
    soft_roc = float(interp(13.0, [9., 10., 15., 25.], [0.055, 0.055, 0.0425, 0.009]))
    self.assertLessEqual(abs(r.path_angle), soft_roc + 1e-9)


if __name__ == '__main__':
  unittest.main()
