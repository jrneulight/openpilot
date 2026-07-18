# Ford Lateral Plan — 2021 Explorer on bp-7.0

**Rewritten:** 2026-07-18 for the fresh bp-7.0 fork (<your-github>/openpilot = BluePilotDev/bluepilot
@ `19858f2888`). Supersedes the bp-6.0 version; all bp-6.0 branches/patches/tools were
deliberately discarded.

**Vehicle:** 2021 Ford Explorer (`FORD_EXPLORER_MK6`, plain CAN / Q3, NOT CAN-FD)
**Device:** comma 3X, dongle `<DONGLE_ID>`

---

## 1. Established findings (validated on real logs, June-July 2026)

These were measured on bp-6.0-era rlogs and remain true of the car regardless of branch:

1. **The car's yaw-rate sensor is hardware-faulty.** `Yaw_Data_FD1.VehYaw_W_Actl` (from the
   RCM) is sign-inverted ~83% of turns, median ratio −0.47 vs truth, ~26% dropout — while
   its CAN quality flag/checksum still read good. Proven against two independent healthy
   references: comma IMU (corr +0.963 vs steering geometry) and pinion angle (corr +0.99).
2. **Everything else in the lateral chain is healthy.** With IMU-based measurement:
   software transmits exactly what the planner asks (0.0% gap), EPS delivers commands at
   ratio ~1.015 (achieved 2.67 m/s² when asked 2.54). No "lazy EPS", no rate starvation.
3. **The historical under-35 mph "steering limit exceeded" bug** was the yaw-fed
   curvature-error clamp acting on inverted data. Replay of the panda check semantics vs
   real logs: 55% fault rate at 20-35 mph with yaw source; ~0% with pinion source at a
   0.003 band. (Below ~20 mph the check is speed-gated off; above ~35 mph curve demands
   are small enough to fit the corrupted band.)
4. **Repair path:** RCM `LB5T-14B321-PA` (~$125-160 used, under center console); FORScan
   PMI procedure exists. DTCs to look for: C0063 (yaw), C0061/62/64 cluster, `:68`
   subcodes. AdvanceTrac/ESC consumes the same signal — fix is warranted independent of
   openpilot.

## 1b. Route `00000001--255eabb4d0` (2026-07-18, first stock-bp-7.0 drive) — key evidence

79-min drive on pristine bp-7.0 (`19858f2888`), MADS active for most of it, max 49 mph.
Findings (all rlog-measured):

1. **Yaw sensor still broken, third independent confirmation on a new day/branch:**
   corr(k_yaw, k_ang) = −0.978; yaw-vs-IMU sign disagreement 86.3%; IMU still healthy
   (corr +0.992 vs geometry). Fault is stable/persistent, not intermittent-by-drive.
2. **bp-7.0 defaults to ANGLE-PRIMARY lateral** (`PrimaryLateralControl.angle`,
   `lateral_angle_ext.py`): on the wire, curvature/path_offset/curv_rate ≈ 0 and
   **path_angle carries all steering** (corr −0.966 vs desired curvature). This is why
   there were no "steering limit exceeded" alerts: panda's path-angle check
   (`FORD_PATH_ANGLE_LIMITS`) has a much wider effective band than the old curvature
   check, and the curvature signal itself is ~0 so the curvature-error check never sees
   a violating command. Only 10 steerSaturated events in 79 min; no fault cascade.
3. **BUT the broken yaw still corrupts control on this branch** — two places:
   - `lateral_angle_ext.py:430-435`: kappa_cmd is clipped to
     `(-yawRate/v) ± CURVATURE_ERROR` above 9 m/s → with inverted yaw the command is
     pinned near/below zero in real curves. Measured on this route: the clip bit on
     ~100% of engaged curve samples at 9-50 mph; |des|−|cmd_κ| median gap 0.00315.
   - `lateral_angle_ext.py:515-540`: the **post-override stall-blip detector** uses
     `desired − current_curvature(yaw)`; with inverted yaw the "stall gap" is inflated
     ~2x permanently → mode-0 blips can fire spuriously in ordinary curves (up to
     3 per episode), each dropping steering authority for ~300 ms + ramp-back.
4. **Net effect is degraded tracking, not faults:** achieved/desired ratio median
   0.88, under-tracking 26.6% of curve samples (healthy IMU-measured reference from
   d9-era logs: ~1.0 ratio, ~1%). The failure mode changed from bp-6.0's hard
   "steering limit exceeded" to bp-7.0's soft "rides wide / weaker in curves".
   933 steerOverride events suggests frequent manual correction.

**Conclusion:** driving stock bp-7.0 is not alert-fatal on this car (angle-primary
masks the old bug) but lateral quality is measurably crippled by the same sensor, and
the yaw-fed sites now number THREE (see §3 updated list).

## 2. ⚠ bp-7.0 status: the yaw-based checks are BACK

bp-7.0 upstream removed the bp-6.0-era 9999 workarounds:

- `opendbc_repo/opendbc/car/ford/carcontroller.py:44` — `if v_ego_raw > 9:` clamp ACTIVE,
  fed by `-CS.out.yawRate` (:142). Same in the BP 4-signal path:
  `opendbc_repo/opendbc/sunnypilot/car/ford/lateral_curv_ext.py:74` (gate > 9) and :288
  (yaw-based current_curvature).
- `opendbc_repo/opendbc/safety/modes/ford.h:124` — `.angle_error_min_speed = 10.0` ACTIVE;
  angle_meas still yaw-sourced (:399-406).

**Consequence: stock bp-7.0 on this car will reproduce the 20-35 mph steering-limit bug**
(in softened, angle-mode form — see §1b). RESOLVED on branch `bp-7.0-pinion` (§3);
the `bp-7.0` branch itself remains a pristine upstream mirror.

Other bp-7.0 deltas noted vs bp-6.0: FORD_LIMITS rate tables now symmetric
(0.0025/0.0014/0.00018 up and down, ford.h:113-120) with a `.frequency = 20U` field;
BP lateral was refactored into `lateral_curv_ext.py` (pc_blend defaults still 0.40/0.40 at
:139-142); anti_overshoot in the stock path is gated to Bronco Sport/F-150 (matches
upstream commaai).

## 3. The fixes — ✅ IMPLEMENTED 2026-07-18 on branch `bp-7.0-pinion`

Both layers now measure curvature from the STEERING PINION ANGLE (SteeringPinion_Data,
0x7E, PSCM) instead of the broken yaw sensor. Deferral reversed by owner same day
("Let's work on the pinion sourced now"; full Fix A companion chosen over interim disables).

**Implemented (commits on bp-7.0-pinion):**
- Python: `get_current_curvature()` helper on LateralCurvExt (VM + liveParameters
  offset/roll); all 3 yaw sites swapped (carcontroller.py, lateral_curv_ext.py,
  lateral_angle_ext.py — including the stall-blip detector's gap input).
- Firmware: `angle_meas` now pinion-sourced in ford_rx_hook; FORD_EXPLORER_PINION_PARAMS
  (slip −5.545e-4 via calc_slip_factor, SR 16.8, WB 3.025, fork-only Explorer-hardcoded);
  QF-gated (StePinCompAnEst_D_Qf==3), counter-checked (0-15), checksum ignored (OEM
  algorithm unknown — tested sum/invert/xor patterns don't match real frames);
  max_angle_error 100→150 (0.003); gate stays 10 m/s. Serves BOTH curvature-mode
  steer_angle_cmd_checks and angle-mode ford_shadow_curvature_error_check (shared sink).
- Bit extraction ground-truthed vs CANParser on 3000/3000 real frames:
  angle `((d[2]&0x7F)<<8)|d[3]`, QF `(d[5]>>2)&0x3`, counter `(d[5]>>4)&0xF`, 99.9 Hz.
- Safety tests: pinion helpers + updated measurement tests + new sign-convention /
  QF-rejection / below-gate tests. Suite: 18 distinct failures vs 19 baseline
  (all pre-existing staleness; zero new failures, one baseline failure now passes).

**Replay validation (real rlogs through exact new firmware semantics):**
| Route | mode | fault rate (new pinion check) | historical yaw ref |
|---|---|---|---|
| 00000001 (bp-7.0) | angle (shadow check) | 0.28% overall / 0.55% @22-35mph | — |
| 00000001 | curvature frames | 0.00% | — |
| d9 (bp-6.0 era) | curvature | **0.018%** | 55% @20-35mph |
Python clip-bite with VM measurement: 4.3-6.0% (vs 14.9-17.7% with broken yaw;
remaining bite is real curve-entry lead, the clip's intended function — and the VM
number here is conservative, computed without the liveParameters offset the real
code applies).

### Implementation details (as landed; supersedes the earlier deferred spec)
- Python helper `get_current_curvature(CS)` lives on `LateralCurvExt` (which owns
  `self.VM`, live-updated from liveParameters, and `self.lp`); reachable from all three
  call sites via CarController's multiple inheritance. Sign per upstream
  `latcontrol_torque.py`. `self.lp is None` falls back to zero offset/roll (replay:
  residuals fit the band even uncorrected).
- The `> 9` gates and Python `CURVATURE_ERROR = 0.002` were kept exactly as upstream.
- Firmware checksum note: `StePinAn_No_Cs` exists in the DBC but its algorithm could not
  be reversed from real frames (Ford sum-invert, XOR, and field-sum variants all fail);
  the message is integrity-checked via its 0-15 counter + quality flag + 100 Hz rx check
  instead (`.ignore_checksum = true`, precedent: FORD_EngVehicleSpThrottle2).
- carstate.py already consumes the pinion signal for steeringAngleDeg and gates
  `vehicleSensorsInvalid` on the same quality flag — the pinion sensor is already
  load-bearing in stock code; safety now simply agrees with control on the source.

### Fix B — pc_blend split (optional, comfort) — NOT implemented
`lateral_curv_ext.py:139-142`: low 0.40→0.50 (straights, road-test-only benefit),
high 0.40→0.20 (curves; bp-6.0-era replay: tracking gap halves). UI custom-profile
params override, so it is testable from the device without code. Revisit after the
pinion branch has road-test mileage (one change per test).

### Cancelled / rejected (do not revisit without new data)
- Closed-loop curvature trim: no tracking deficit exists (EPS ratio 1.015).
- Raising CURVATURE_MAX / panda 0.012 / MAX_LATERAL_ACCEL: never binding for this car
  (accel clamp is CAN-FD-only dead code on Explorer).
- Precision-mode flip: already Precise; community data says Comfortable trades away
  curvature rate.
- Interim 9999 disables: superseded by the implemented pinion source (both layers).

## 4. Sequencing (updated 2026-07-18: pinion fix landed)

1. ✅ Pinion-sourced measurement in both layers — implemented on `bp-7.0-pinion`,
   replay-validated, safety tests not-worse-than-baseline.
2. **Road test** (owner): install `installer.comma.ai/<your-github>/bp-7.0-pinion`
   (panda auto-reflashes). Expect: no steering-limit alerts, no panda blocks
   (`pandaStates`), tracking ratio → ~1.0, stall-blips only on genuine stalls.
   Re-run the measurement loop (§5) on the uploaded route and compare vs §1b numbers.
3. Fix B pc_blend params from the device UI, if wanted, after step 2 is clean.
4. RCM repair when convenient → optionally revert to pure upstream (yaw-sourced), or
   keep the pinion source (it is arguably the better measurement on any Ford: same
   sensor family Tesla/Toyota/Nissan safety uses, and immune to this RCM failure class).

## 5. Measurement / validation loop

`tools/ford_pinion_replay.py` (committed on bp-7.0-pinion) replays the firmware check
semantics and Python clip against rlogs — set `FORD_REPLAY_DONGLE_ID` env var to your
dongle ID and edit the route list. The older bp-6.0 analysis tools were deleted with the
branch reset; rebuild from this recipe if needed:
- qlogs: `LogReader(f'{route}/{seg}/a')`; rlogs when uploaded. comma auth in `.venv`
  (auth.py google; account <comma account email>).
- desired = `carControl.actuators.curvature`; commanded = `LatCtlCurv_No_Actl` decoded
  from `sendcan` msg 979 (sign-inverted vs openpilot convention); achieved =
  `-livePose.angularVelocityDevice.z / vEgo` (NEVER `carState.yawRate` on this car);
  geometry cross-check = `radians(steeringAngleDeg) / (16.8 * 3.025)` slip-corrected.
- engagement flag = `selfdriveStateSP.mads.active` (NOT carControl.latActive on this fork).
- Baseline reference numbers to beat are in §1 and §3.

## 6. Standing constraints

- Treat `carState.yawRate` / `Yaw_Data_FD1` as permanently untrusted on this car until
  the RCM is replaced AND revalidated (intermittently-plausible failure mode).
- One change per road test; hands ready; revert on any "steering limit exceeded" or
  panda-blocked messages (`pandaStates`).
- Safety-layer edits only as specified here (validated values); never loosen beyond them.
