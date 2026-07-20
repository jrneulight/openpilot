"""
BluePilot: Ford crawl-speed experiment lab (EXPERIMENT BRANCH ONLY).

Purpose: determine, on-car, whether ANY wire-level lever produces steering motion in
the crawl regime where the Q3 PSCM has measured 0.00-0.21x execution of normal
commands (stop-and-go drift attribution, route 0x33): is the deadband signal
starvation (the kappa*v mapping under-signals at crawl -> bigger path_angle would
move the wheel), static friction (a dither breaks it), a distinct path_offset
response (the PSCM may honor lateral-offset requests while ignoring heading
requests), or an absolute policy floor (nothing moves, case closed with proof).

When enabled (FordCrawlLabEnable, default OFF) and ONLY while engaged, hands-off,
below _LAB_MAX_V_MS, the lab overrides the wire path_angle/path_offset with a
repeating phase pattern. The wire records the stimulus (LMC LatCtlPath_An_Actl /
LatCtlPathOffst_L_Actl) and the pinion records the response, so a single supervised
parking-lot session answers every lever at once -- no extra telemetry needed.

Safety: pattern targets are far inside the panda's value limits (path_angle +-0.5
rad, path_offset +-1.0 m) and every transition is slew-bounded well under normal
control rates; below 5 m/s the panda's angle-error check is inert by design, and the
value/ROC checks remain enforced. A driver press releases the override IMMEDIATELY
(normal control resumes on the same tick); exceeding the speed gate slews the
pattern back to zero first. Supervised empty-lot use only.
"""

_LAB_MAX_V_MS = 4.4     # active only below this (panda error-check floor is 5.0)
_LAB_PA_SLEW = 0.0075   # rad per 20 Hz tick = 0.15 rad/s (normal control unwinds at 0.40)
_LAB_PO_SLEW = 0.0125   # m per 20 Hz tick = 0.25 m/s
_DITHER_AMP = 0.05      # rad; slew-shaping turns the sine into a ~2 Hz triangle, still a dither

# (name, duration_s, path_angle_target_rad, path_offset_target_m, dither_hz)
PHASES = (
  ('baseline',  3.0,  0.00, 0.0, 0.0),
  ('pa+0.05',   3.0,  0.05, 0.0, 0.0),
  ('zero',      2.0,  0.00, 0.0, 0.0),
  ('pa-0.05',   3.0, -0.05, 0.0, 0.0),
  ('zero',      2.0,  0.00, 0.0, 0.0),
  ('pa+0.12',   3.0,  0.12, 0.0, 0.0),
  ('zero',      2.0,  0.00, 0.0, 0.0),
  ('pa-0.12',   3.0, -0.12, 0.0, 0.0),
  ('zero',      2.0,  0.00, 0.0, 0.0),
  ('pa+0.20',   4.0,  0.20, 0.0, 0.0),
  ('zero',      2.0,  0.00, 0.0, 0.0),
  ('pa-0.20',   4.0, -0.20, 0.0, 0.0),
  ('zero',      2.0,  0.00, 0.0, 0.0),
  ('dither',    4.0,  0.00, 0.0, 2.0),
  ('zero',      2.0,  0.00, 0.0, 0.0),
  ('po+0.5',    4.0,  0.00, 0.5, 0.0),
  ('zero',      2.0,  0.00, 0.0, 0.0),
  ('po-0.5',    4.0,  0.00, -0.5, 0.0),
  ('zero',      2.0,  0.00, 0.0, 0.0),
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
    self.enabled = False       # FordCrawlLabEnable, refreshed by update_angle_params
    self.active = False        # overriding the wire this tick
    self._phase_i = 0
    self._phase_t = 0.0
    self._pa = 0.0
    self._po = 0.0

  def update(self, engaged_hands_free: bool, pressed: bool, v_ego: float, dt: float) -> tuple[float, float]:
    """Advance one 20 Hz tick; returns (path_angle, path_offset) to put on the wire.

    Only meaningful when self.active is True afterwards. A press drops the override
    on THIS tick (driver first; the wire step is the panda's value-check territory,
    harmless at crawl with the driver holding the wheel). Losing any other condition
    slews the outputs back to zero before deactivating, so the hand-back is smooth.
    """
    conditions = self.enabled and engaged_hands_free and not pressed and v_ego < _LAB_MAX_V_MS
    if pressed or not self.enabled:
      self.active = False
      self._phase_i = 0
      self._phase_t = 0.0
      self._pa = 0.0
      self._po = 0.0
      return 0.0, 0.0
    if not conditions:
      # smooth ramp-out, stay in control of the wire until settled at zero
      self._phase_i = 0
      self._phase_t = 0.0
      self._pa = _toward(self._pa, 0.0, _LAB_PA_SLEW)
      self._po = _toward(self._po, 0.0, _LAB_PO_SLEW)
      self.active = abs(self._pa) > 1e-4 or abs(self._po) > 1e-3
      return self._pa, self._po

    self.active = True
    name, dur, pa_t, po_t, dither_hz = PHASES[self._phase_i]
    self._phase_t += dt
    if self._phase_t >= dur:
      self._phase_t = 0.0
      self._phase_i = (self._phase_i + 1) % len(PHASES)
    if dither_hz > 0.0:
      import math
      pa_t = _DITHER_AMP * math.sin(2.0 * math.pi * dither_hz * self._phase_t)
    self._pa = _toward(self._pa, pa_t, _LAB_PA_SLEW)
    self._po = _toward(self._po, po_t, _LAB_PO_SLEW)
    return self._pa, self._po
