"""
BluePilot: Ford crawl-speed experiment lab (EXPERIMENT BRANCH ONLY).

Purpose: determine, on-car, whether ANY wire-level lever produces steering motion in
the crawl regime where the Q3 PSCM has measured 0.00-0.21x execution of normal
commands (stop-and-go drift attribution, route 0x33). The full protocol surface is
swept -- angle mode normally transmits a DEGENERATE signal set (curvature,
curvature_rate and path_offset all zero, only path_angle live), which stock never
does, so the lab also exercises the levers angle mode never touches:

  - path_angle magnitude steps (0.05/0.12/0.20 rad; bypasses the kappa*v starvation)
  - a 10 s long hold (catches a slow integrator)
  - 2 Hz and 4 Hz dithers, and dither SUPERIMPOSED on a step (break static friction
    WHILE requesting a direction -- the canonical stiction pattern)
  - path_offset requests (+-0.5 m; the PSCM may honor lateral offset while ignoring
    heading)
  - CURVATURE signal steps (+-0.01/0.02; the PSCM's primary actuator in stock
    curvature-mode operation)
  - a COHERENT full set (curvature + path_angle + path_offset describing one
    consistent curve -- tests whether the PSCM rejects degenerate sets at crawl)
  - precision-mode and ramp-type variants of the same step
  - a mode-0 reset pulse immediately before a step (fresh-authority test)

Consciously excluded, with reasons: the LKA-aid intervention channel (the PSCM's own
LaActAvail matrix declares LKA suppressed at crawl, same bit as LCA -- phase-2 only
if this round nulls) and speeds above 4.4 m/s (the panda angle-error check wakes at
5 m/s and would fight large probes; the 4.4-9 m/s band is reachable by ordinary
comp-table iteration once the mechanism is known).

The wire records every stimulus (LMC curvature/path_angle/path_offset fields) and
the pinion records every response; the rlog is the complete dataset.

Safety: FordCrawlLabEnable default OFF; active ONLY engaged + hands-off + below
_LAB_MAX_V_MS; targets far inside panda value limits (path_angle +-0.5 rad, offset
+-1.0 m, curvature +-0.02); every transition slew-bounded well under normal control
rates; a driver press releases the override on the SAME tick; losing the speed gate
ramps out smoothly first. Supervised empty-lot use only.
"""
import math

_LAB_MAX_V_MS = 4.4     # active only below this (panda error-check floor is 5.0)
_LAB_PA_SLEW = 0.0075   # rad per 20 Hz tick = 0.15 rad/s (normal control unwinds at 0.40)
_LAB_PO_SLEW = 0.0125   # m per tick = 0.25 m/s
_LAB_CURV_SLEW = 0.001  # 1/m per tick (panda crawl ROC allows 0.0025)


def _phase(name, dur, pa=0.0, po=0.0, curv=0.0, dither_hz=0.0, dither_amp=0.0,
           precision=None, ramp_type=None, blip=False):
  return dict(name=name, dur=dur, pa=pa, po=po, curv=curv, dither_hz=dither_hz,
              dither_amp=dither_amp, precision=precision, ramp_type=ramp_type, blip=blip)


_Z = _phase('zero', 2.0)
PHASES = (
  _phase('baseline', 3.0),
  # -- path_angle magnitude ladder --
  _phase('pa+0.05', 3.0, pa=0.05), _Z, _phase('pa-0.05', 3.0, pa=-0.05), _Z,
  _phase('pa+0.12', 3.0, pa=0.12), _Z, _phase('pa-0.12', 3.0, pa=-0.12), _Z,
  _phase('pa+0.20', 4.0, pa=0.20), _Z, _phase('pa-0.20', 4.0, pa=-0.20), _Z,
  # -- slow integrator --
  _phase('pa_hold+0.12', 10.0, pa=0.12), _Z,
  # -- stiction breakers --
  _phase('dither2', 4.0, dither_hz=2.0, dither_amp=0.05), _Z,
  _phase('dither4', 4.0, dither_hz=4.0, dither_amp=0.04), _Z,
  _phase('step+dither', 5.0, pa=0.12, dither_hz=2.0, dither_amp=0.03), _Z,
  # -- the lever angle mode never uses: lateral offset --
  _phase('po+0.5', 4.0, po=0.5), _Z, _phase('po-0.5', 4.0, po=-0.5), _Z,
  # -- the PSCM's stock primary actuator: curvature --
  _phase('curv+0.01', 4.0, curv=0.01), _Z, _phase('curv-0.01', 4.0, curv=-0.01), _Z,
  _phase('curv+0.02', 4.0, curv=0.02), _Z,
  # -- coherent full set: one consistent curve at crawl --
  _phase('coherent', 6.0, pa=0.10, po=0.25, curv=0.015), _Z,
  # -- PSCM servo-mode variants of the same step --
  _phase('precision0', 4.0, pa=0.12, precision=0), _Z,
  _phase('ramp0', 4.0, pa=0.12, ramp_type=0), _phase('ramp3', 4.0, pa=0.12, ramp_type=3), _Z,
  # -- fresh authority: mode-0 reset pulse on entry, then the step --
  _phase('blip_then_pa', 5.0, pa=0.12, blip=True), _Z,
)


def _toward(cur: float, target: float, step: float) -> float:
  d = target - cur
  if d > step:
    d = step
  elif d < -step:
    d = -step
  return cur + d


class CrawlLab:
  def __init__(self):
    self.enabled = False        # FordCrawlLabEnable, refreshed by update_angle_params
    self.active = False         # overriding the wire this tick
    self.precision = None       # per-phase LatCtlPrecision override (None = leave normal)
    self.ramp_type = None       # per-phase LatCtlRampType override (None = leave normal)
    self.request_blip = False   # one-shot: integration fires a mode-0 pulse when True
    self._phase_i = 0
    self._phase_t = 0.0
    self._phase_entered = False
    self._pa = 0.0
    self._po = 0.0
    self._curv = 0.0

  def notify_mode0(self):
    """A mode-0 frame is on the wire (blip/human-turn): the stimulus restarts from zero,
    so the post-pulse step is a clean slew from rest rather than a jump to a stale value."""
    self._pa = 0.0
    self._po = 0.0
    self._curv = 0.0

  def _idle(self):
    self._phase_i = 0
    self._phase_t = 0.0
    self._phase_entered = False
    self.precision = None
    self.ramp_type = None
    self.request_blip = False

  def update(self, engaged_hands_free: bool, pressed: bool, v_ego: float, dt: float) -> tuple[float, float, float]:
    """Advance one 20 Hz tick; returns (path_angle, path_offset, curvature) for the wire.

    Only meaningful while self.active. A press drops the override THIS tick (driver
    first); losing any other condition slews all outputs to zero before deactivating.
    """
    conditions = self.enabled and engaged_hands_free and not pressed and v_ego < _LAB_MAX_V_MS
    if pressed or not self.enabled:
      self.active = False
      self._idle()
      self._pa = self._po = self._curv = 0.0
      return 0.0, 0.0, 0.0
    if not conditions:
      self._idle()
      self._pa = _toward(self._pa, 0.0, _LAB_PA_SLEW)
      self._po = _toward(self._po, 0.0, _LAB_PO_SLEW)
      self._curv = _toward(self._curv, 0.0, _LAB_CURV_SLEW)
      self.active = abs(self._pa) > 1e-4 or abs(self._po) > 1e-3 or abs(self._curv) > 1e-5
      return self._pa, self._po, self._curv

    self.active = True
    ph = PHASES[self._phase_i]
    if not self._phase_entered:
      self._phase_entered = True
      self.request_blip = bool(ph['blip'])   # one-shot on phase entry
    else:
      self.request_blip = False
    self.precision = ph['precision']
    self.ramp_type = ph['ramp_type']
    self._phase_t += dt
    if self._phase_t >= ph['dur']:
      self._phase_t = 0.0
      self._phase_entered = False
      self._phase_i = (self._phase_i + 1) % len(PHASES)
    pa_t = ph['pa']
    if ph['dither_hz'] > 0.0:
      pa_t = pa_t + ph['dither_amp'] * math.sin(2.0 * math.pi * ph['dither_hz'] * self._phase_t)
    self._pa = _toward(self._pa, pa_t, _LAB_PA_SLEW)
    self._po = _toward(self._po, ph['po'], _LAB_PO_SLEW)
    self._curv = _toward(self._curv, ph['curv'], _LAB_CURV_SLEW)
    return self._pa, self._po, self._curv
