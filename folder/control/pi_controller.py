import json
import time
from pathlib import Path

_PROJECT_ROOT = Path(__file__).resolve().parent.parent


# ── Config loader ──────────────────────────────────────────────────────────────

def load_control_config(path: str = "config/control_config.json") -> dict:
    """
    Load control parameters from JSON file.
    Called fresh every control cycle so changes take effect without restart.
    """
    resolved = str(_PROJECT_ROOT / path)
    with open(resolved) as f:
        return json.load(f)


# ── PI Controller ──────────────────────────────────────────────────────────────

class PIController:
    """
    Discrete PI controller with:
      - configurable Kp, Ki, output limits
      - ramp rate limiting (separate up/down rates)
      - anti-windup via integral correction when ramp clamps the output
    """

    def __init__(self):
        self.integral      = 0.0
        self.prev_time     = None
        self.prev_cmd_kw   = 0.0    # last ramped output, used by ramp limiter

    def reset(self):
        """Reset controller state — call if there is a gap in measurements."""
        self.integral    = 0.0
        self.prev_time   = None
        self.prev_cmd_kw = 0.0

    def update(self, measured_kw: float, config: dict) -> tuple:
        """
        Run one PI cycle.

        Args:
            measured_kw : current measured power from meter (kW)
            config      : dict loaded from control_config.json

        Returns:
            (percent, ramped_kw)
              percent   : value to write to c_pac register (0.0 – 100.0)
              ramped_kw : ramp-limited PI output in kW (for logging)

        Returns (None, None) if inside deadband — caller should skip write.
        """
        setpoint_kw       = config["setpoint_kw"]
        total_capacity_kw = config["total_capacity_kw"]
        kp                = config["kp"]
        ki                = config["ki"]
        deadband_kw       = config["deadband_kw"]
        ramp_up_kw_per_s  = config["ramp_up_kw_per_sec"]
        ramp_dn_kw_per_s  = config["ramp_down_kw_per_sec"]

        # ── Timing ────────────────────────────────────────────────────────────
        now = time.monotonic()
        dt  = 1.0 if self.prev_time is None else now - self.prev_time
        self.prev_time = now

        # ── Deadband check ────────────────────────────────────────────────────
        error_kw = setpoint_kw - measured_kw
        print(f"[PI] Setpoint={setpoint_kw:.2f} kW  Measured={measured_kw:.3f} kW  "
              f"Error={error_kw:+.3f} kW  Deadband=±{deadband_kw} kW")

        if abs(error_kw) <= deadband_kw:
            print(f"[PI] Inside deadband — holding prev_cmd={self.prev_cmd_kw:.3f} kW")
            return None, None

        # ── PI output ─────────────────────────────────────────────────────────
        self.integral += error_kw * dt
        raw_kw         = kp * error_kw + ki * self.integral

        # Clamp PI output to [0, total_capacity_kw]
        cmd_kw = max(0.0, min(total_capacity_kw, raw_kw))

        print(f"[PI] dt={dt:.3f}s  integral={self.integral:.4f}  "
              f"P={kp * error_kw:.4f}  I={ki * self.integral:.4f}  "
              f"raw={raw_kw:.4f}  clamped={cmd_kw:.4f} kW")

        # ── Ramp limiter ──────────────────────────────────────────────────────
        ramp_up = ramp_up_kw_per_s * dt
        ramp_dn = ramp_dn_kw_per_s * dt
        delta   = cmd_kw - self.prev_cmd_kw

        if delta >= 0:
            ramped_kw = self.prev_cmd_kw + min(delta, ramp_up)
        else:
            ramped_kw = self.prev_cmd_kw - min(abs(delta), ramp_dn)

        # ── Anti-windup ───────────────────────────────────────────────────────
        # If ramp clamped the output, correct integral to match actual output
        if ramped_kw != cmd_kw:
            self.integral = (ramped_kw - kp * error_kw) / ki
            print(f"[PI] Anti-windup: integral corrected to {self.integral:.4f}")

        print(f"[RAMP] cmd={cmd_kw:.3f}  prev={self.prev_cmd_kw:.3f}  "
              f"delta={delta:+.3f}  up={ramp_up:.2f}  dn={ramp_dn:.2f}  "
              f"ramped={ramped_kw:.3f} kW")

        self.prev_cmd_kw = ramped_kw

        # ── Convert to percent ────────────────────────────────────────────────
        percent = (ramped_kw / total_capacity_kw) * 100.0
        percent = max(0.0, min(100.0, percent))

        print(f"[PI] c_pac={percent:.4f}%")

        return percent, ramped_kw
