import time
from pymodbus.client import ModbusTcpClient
import threading

from flask import Flask, request, jsonify

app = Flask(__name__)

# ── Meter (read PTOT) ──────────────────────────────────────────
METER_HOST      = "192.168.178.52"
METER_PORT      = 502
METER_DEVICE_ID = 1

# ── Inverter (write C_Pac) ─────────────────────────────────────
INV_HOST        = "192.168.178.51"
INV_PORT        = 502
INV_DEVICE_ID   = 2

# ── Sentinel values ────────────────────────────────────────────
SENTINEL_U16 = 0xFFFF
SENTINEL_U32 = 0xFFFFFFFF

# ── Meter register map for PTOT ────────────────────────────────
# SMA Energy Meter / Sunny Home Manager typically exposes PTOT
# as a 32-bit signed value in W at registers 30775/30776.
# Adjust addresses below if your meter differs.
PTOT_HI_ADDR = 203
PTOT_LO_ADDR = 204
PTOT_DTYPE   = "S32"
PTOT_SCALE   = 1000  # raw unit is W; we divide by 1000 to get kW

# Switch to Config File for flask

CONTROL_CONFIG = {
    "setpoint_kw": 110.0,
    "total_capacity_kw": 880.0,
    "kp": 0.6,
    "ki": 0.03,
    "deadband_kw" : 10.0,
    "ramp_up_kw_per_sec": 50.0,
    "ramp_down_kw_per_sec": 10.0
}

# ── Control constants ──────────────────────────────────────────


C_PAC_ADDR        = 40023    # inverter curtailment register (%)
LOOP_INTERVAL     = 1      # seconds between cycles
# ══════════════════════════════════════════════════════════════
#  Helpers
# ══════════════════════════════════════════════════════════════

def to_s32(value: int) -> int:
    return value - 0x100000000 if value & 0x80000000 else value


def decode_register(data: dict, hi_addr: int, lo_addr, dtype: str):
    """Decode a 16- or 32-bit register value from a raw data dict."""
    if hi_addr not in data:
        print(f"    [DECODE] hi_addr {hi_addr} not in data dict — missing block?")
        return None

    # 16-bit
    if lo_addr is None:
        raw = data[hi_addr]
        print(f"    [DECODE] 16-bit raw @ {hi_addr} = {raw:#06x} ({raw})")
        if raw == SENTINEL_U16:
            print(f"    [DECODE] Sentinel U16 detected — returning None")
            return None
        if dtype == "S16":
            val = raw - 0x10000 if raw & 0x8000 else raw
            print(f"    [DECODE] S16 interpreted = {val}")
            return val
        return raw

    # 32-bit
    if lo_addr not in data:
        print(f"    [DECODE] lo_addr {lo_addr} not in data dict")
        return None
    raw = (data[hi_addr] << 16) | data[lo_addr]
    print(f"    [DECODE] 32-bit raw @ {hi_addr}/{lo_addr} = {raw:#010x} ({raw})")
    if raw == SENTINEL_U32:
        print(f"    [DECODE] Sentinel U32 detected — returning None")
        return None
    if dtype == "S32":
        val = to_s32(raw)
        print(f"    [DECODE] S32 interpreted = {val}")
        return val
    return raw


# ══════════════════════════════════════════════════════════════
#  Meter — read PTOT
# ══════════════════════════════════════════════════════════════

def read_ptot_kw(meter_client: ModbusTcpClient):
    """
    Read total active power (PTOT) from the energy meter.
    Returns float in kW, or None on failure.
    """
    print(f"\n[METER] Reading PTOT registers {PTOT_HI_ADDR}/{PTOT_LO_ADDR} "
          f"from {METER_HOST}:{METER_PORT} device {METER_DEVICE_ID}")

    # Read a small block covering both registers
    start = min(PTOT_HI_ADDR, PTOT_LO_ADDR)
    count = abs(PTOT_LO_ADDR - PTOT_HI_ADDR) + 1

    result = meter_client.read_holding_registers(
        address=start,
        count=count,
        device_id=METER_DEVICE_ID,
    )

    if result.isError():
        print(f"[METER] ERROR reading registers: {result}")
        return None

    print(f"[METER] Raw register dump (start={start}, count={count}):")
    data = {}
    for i, val in enumerate(result.registers):
        addr = start + i
        data[addr] = val
        print(f"    addr {addr:5d} = {val:#06x}  ({val})")

    raw = decode_register(data, PTOT_HI_ADDR, PTOT_LO_ADDR, PTOT_DTYPE)

    if raw is None:
        print("[METER] PTOT decode returned None")
        return None

    ptot_w  = raw * PTOT_SCALE
    ptot_kw = ptot_w / 1000.0
    print(f"[METER] PTOT = {ptot_w} W  →  {ptot_kw:.3f} kW")
    return ptot_kw


# ══════════════════════════════════════════════════════════════
#  Inverter — write C_Pac
# ══════════════════════════════════════════════════════════════

def write_cpac_percent(inv_client: ModbusTcpClient, percent: float) -> bool:
    """
    Write curtailment % to C_Pac register on the inverter.
    C_Pac scale = 0.01  →  raw = percent / 0.01
    Register dtype S16, sent as unsigned 16-bit word.
    """
    raw_value = int(round(percent / 0.01))
    raw_value = max(-32768, min(32767, raw_value))  # clamp to S16
    word      = raw_value & 0xFFFF                  # send as unsigned word

    print(f"\n[INVERTER] Writing C_Pac: {percent:.4f}%  →  raw {raw_value} ({word:#06x})"
          f" to register {C_PAC_ADDR} on {INV_HOST}:{INV_PORT} device {INV_DEVICE_ID}")

    result = inv_client.write_register(
        address=C_PAC_ADDR,
        value=word,
        device_id=INV_DEVICE_ID,
    )

    if result.isError():
        print(f"[INVERTER] ERROR writing register: {result}")
        return False

    print(f"[INVERTER] Write successful.")
    return True


# ══════════════════════════════════════════════════════════════
#  PI Controller
# ══════════════════════════════════════════════════════════════

class PIController:
    def __init__(self, kp: float, ki: float,
                  output_limits=(0.0, float("inf"))):
        self.kp            = kp
        self.ki            = ki
        self.integral      = 0.0
        self.prev_time     = None
        self.output_limits = output_limits

    def update(self, setpoint_kw: float, measured_kw: float) -> float:
        now = time.time()
        dt  = LOOP_INTERVAL if self.prev_time is None else now - self.prev_time
        self.prev_time = now

        error          = setpoint_kw - measured_kw
        self.integral += error * dt
        output_kw      = self.kp * error + self.ki * self.integral

        min_out, max_out = self.output_limits
        clamped = max(min_out, min(max_out, output_kw))

        print(f"\n[PI]  dt={dt:.3f}s")
        print(f"[PI]  error         = {setpoint_kw:.2f} - {measured_kw:.3f} = {error:+.3f} kW")
        print(f"[PI]  integral      = {self.integral:.4f}")
        print(f"[PI]  P term        = {self.kp} * {error:+.3f} = {self.kp * error:.4f} kW")
        print(f"[PI]  I term        = {self.ki} * {self.integral:.4f} = {self.ki * self.integral:.4f} kW")
        print(f"[PI]  raw output    = {output_kw:.4f} kW")
        print(f"[PI]  clamped output= {clamped:.4f} kW  (limits: {self.output_limits})")

        return clamped

# Flask route setup 

@app.route("/update", methods=["POST"])
def update_config():
    data = request.json

    if "setpoint_kw" in data:
        CONTROL_CONFIG["setpoint_kw"] = float(data["setpoint_kw"])

    if "total_capacity_kw" in data:
        CONTROL_CONFIG["total_capacity_kw"] = float(data["total_capacity_kw"])

    if "kp" in data:
        CONTROL_CONFIG["kp"] = float(data["kp"])

    if "ki" in data:
        CONTROL_CONFIG["ki"] = float(data["ki"])

    if "deadband_kw" in data:
        CONTROL_CONFIG["deadband_kw"] = float(data["deadband_kw"])

    if "ramp_up_kw_per_sec" in data:
        CONTROL_CONFIG["ramp_up_kw_per_sec"] = float(data["ramp_up_kw_per_sec"])

    if "ramp_down_kw_per_sec" in data:
        CONTROL_CONFIG["ramp_down_kw_per_sec"] = float(data["ramp_down_kw_per_sec"])

    return jsonify({"status": "updated", "config": CONTROL_CONFIG})

@app.route("/")
def home():
    return f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Control Panel</title>
        <style>
            body {{
                font-family: Arial, sans-serif;
                background: #f4f6f8;
                display: flex;
                justify-content: center;
                align-items: center;
                height: 100vh;
                margin: 0;
            }}

            .card {{
                background: white;
                padding: 30px;
                border-radius: 12px;
                box-shadow: 0 10px 25px rgba(0,0,0,0.1);
                width: 320px;
            }}

            h2 {{
                text-align: center;
                margin-bottom: 20px;
            }}

            label {{
                font-size: 14px;
                font-weight: bold;
            }}

            input {{
                width: 100%;
                padding: 8px;
                margin-top: 5px;
                margin-bottom: 15px;
                border-radius: 6px;
                border: 1px solid #ccc;
            }}

            button {{
                width: 100%;
                padding: 10px;
                border: none;
                border-radius: 8px;
                background: #007bff;
                color: white;
                font-size: 16px;
                cursor: pointer;
            }}

            button:hover {{
                background: #0056b3;
            }}

            .status {{
                margin-top: 15px;
                text-align: center;
                font-size: 14px;
                color: green;
            }}
        </style>
    </head>
    <body>

        <div class="card">
            <h2>Control Panel</h2>

            <form id="form">
                <label>Setpoint (kW)</label>
                <input type="number" step="0.1" id="setpoint" value="{CONTROL_CONFIG['setpoint_kw']}">

                <label>Total Capacity (kW)</label>
                <input type="number" step="0.1" id="capacity" value="{CONTROL_CONFIG['total_capacity_kw']}">

                <label>Kp</label>
                <input type="number" step="0.01" id="kp" value="{CONTROL_CONFIG['kp']}">

                <label>Ki</label>
                <input type="number" step="0.01" id="ki" value="{CONTROL_CONFIG['ki']}">
                
                <label>Deadband (kW)</label>
                <input type="number" step="0.1" id="deadband" value="{CONTROL_CONFIG['deadband_kw']}">

                <label>Ramp Up (kW/s)</label>
                <input type="number" step="1" id="ramp_up" value="{CONTROL_CONFIG['ramp_up_kw_per_sec']}">

                <label>Ramp Down (kW/s)</label>
                <input type="number" step="1" id="ramp_down" value="{CONTROL_CONFIG['ramp_down_kw_per_sec']}">

                <button type="submit">Update</button>
            </form>

            <div class="status" id="status"></div>
        </div>

        <script>
            document.getElementById("form").onsubmit = async (e) => {{
                e.preventDefault();

                const data = {{
                    setpoint_kw: document.getElementById("setpoint").value,
                    total_capacity_kw: document.getElementById("capacity").value,
                    kp: document.getElementById("kp").value,
                    ki: document.getElementById("ki").value,
                    deadband_kw: document.getElementById("deadband").value,
                    ramp_up_kw_per_sec: document.getElementById("ramp_up").value,
                    ramp_down_kw_per_sec: document.getElementById("ramp_down").value
                }};

                const res = await fetch("/update", {{
                    method: "POST",
                    headers: {{ "Content-Type": "application/json" }},
                    body: JSON.stringify(data)
                }});

                document.getElementById("status").innerText = "Updated successfully!";
            }};
        </script>

    </body>
    </html>
    """
# ══════════════════════════════════════════════════════════════
#  Main
# ══════════════════════════════════════════════════════════════

def main():
    print("=" * 60)
    print("  Modbus PI Power Controller — Debug Mode")
    print("=" * 60)
    print(f"  Meter    : {METER_HOST}:{METER_PORT}  device {METER_DEVICE_ID}")
    print(f"  Inverter : {INV_HOST}:{INV_PORT}  device {INV_DEVICE_ID}")
    # print(f"  Setpoint : {setpoint_kw} kW")
    # print(f"  Capacity : {total_capacity_kw} kW")
    print(f"  C_Pac reg: {C_PAC_ADDR}")
    print("=" * 60)

    # ── Connect to meter ──────────────────────────────────────
    print(f"\n[INIT] Connecting to meter    {METER_HOST}:{METER_PORT} …")
    meter_client = ModbusTcpClient(METER_HOST, port=METER_PORT)
    if not meter_client.connect():
        print(f"[ERROR] Cannot connect to meter at {METER_HOST}:{METER_PORT}")
        return
    print("[INIT] Meter connected OK.")

    # ── Connect to inverter ───────────────────────────────────
    print(f"[INIT] Connecting to inverter {INV_HOST}:{INV_PORT} …")
    inv_client = ModbusTcpClient(INV_HOST, port=INV_PORT)
    if not inv_client.connect():
        print(f"[ERROR] Cannot connect to inverter at {INV_HOST}:{INV_PORT}")
        meter_client.close()
        return
    print("[INIT] Inverter connected OK.")

    controller = PIController(kp=0.6, ki=0.03)
    cycle      = 0

    try:
        prev_cmd_kw = 0.0   # tracks last ramped output for ramp limiter
        print("\n[LOOP] Starting control loop (Ctrl-C to stop)…\n")

        while True:
            cycle += 1
            print(f"\n{'━' * 60}")
            print(f"  CYCLE {cycle}  |  {time.strftime('%H:%M:%S')}")
            print(f"{'━' * 60}")

            setpoint_kw = CONTROL_CONFIG["setpoint_kw"]
            total_capacity_kw = CONTROL_CONFIG["total_capacity_kw"]
            controller.output_limits = (0.0, total_capacity_kw)

            controller.kp = CONTROL_CONFIG["kp"]
            controller.ki = CONTROL_CONFIG["ki"]

            # 1. Read PTOT from meter → this is our measured_kw
            measured_kw = read_ptot_kw(meter_client)

            if measured_kw is None:
                print("[LOOP] Skipping cycle — no PTOT reading.")
                time.sleep(LOOP_INTERVAL)
                continue

            # 2. Error + deadband check
            deadband_kw = CONTROL_CONFIG["deadband_kw"]
            error_kw = setpoint_kw - measured_kw
            print(f"\n[LOOP] Setpoint    : {setpoint_kw:.2f} kW")
            print(f"[LOOP] Measured    : {measured_kw:.3f} kW")
            print(f"[LOOP] Error       : {error_kw:+.3f} kW")
            print(f"[LOOP] Deadband    : ±{deadband_kw} kW")

            if abs(error_kw) <= deadband_kw:
                print(f"[LOOP] Inside deadband — holding C_Pac={prev_cmd_kw:.2f}%, skipping PI.")
                time.sleep(LOOP_INTERVAL)
                continue

            # # 3. PI output

            # cmd_kw = controller.update(setpoint_kw, measured_kw)
            # ramp_up   = CONTROL_CONFIG["ramp_up_kw_per_sec"]   * LOOP_INTERVAL
            # ramp_down = CONTROL_CONFIG["ramp_down_kw_per_sec"]  * LOOP_INTERVAL

            # delta = cmd_kw - prev_cmd_kw
            # if delta >= 0:
            #     ramped_kw = prev_cmd_kw + min(delta, ramp_up)
            # else:
            #     ramped_kw = prev_cmd_kw - min(abs(delta), ramp_down)

            # print(f"[RAMP] PI cmd={cmd_kw:.3f} kW | prev={prev_cmd_kw:.3f} kW | "
            #     f"delta={delta:+.3f} | ramp_up={ramp_up:.2f} ramp_down={ramp_down:.2f} | "
            #     f"ramped={ramped_kw:.3f} kW")
            # prev_cmd_kw = ramped_kw

            # 3. PI output
            cmd_kw = controller.update(setpoint_kw, measured_kw)
            ramp_up   = CONTROL_CONFIG["ramp_up_kw_per_sec"]   * LOOP_INTERVAL
            ramp_down = CONTROL_CONFIG["ramp_down_kw_per_sec"]  * LOOP_INTERVAL

            delta = cmd_kw - prev_cmd_kw
            if delta >= 0:
                ramped_kw = prev_cmd_kw + min(delta, ramp_up)
            else:
                ramped_kw = prev_cmd_kw - min(abs(delta), ramp_down)

            # Anti-windup: if ramp clamped, correct the integral
            if ramped_kw != cmd_kw:
                controller.integral = (ramped_kw - controller.kp * (setpoint_kw - measured_kw)) / controller.ki
                print(f"[ANTI-WINDUP] Ramp clamped — integral corrected to {controller.integral:.4f}")

            print(f"[RAMP] PI cmd={cmd_kw:.3f} kW | prev={prev_cmd_kw:.3f} kW | "
                f"delta={delta:+.3f} | ramp_up={ramp_up:.2f} ramp_down={ramp_down:.2f} | "
                f"ramped={ramped_kw:.3f} kW")
            prev_cmd_kw = ramped_kw

            # 5. Convert to %
            percent = (ramped_kw / total_capacity_kw) * 100.0
            percent = max(0.0, min(100.0, percent))
            print(f"[LOOP] C_Pac       : {percent:.4f}%")

            # 6. Write
            ok = write_cpac_percent(inv_client, percent)

            status = "WRITE OK " if ok else "WRITE ERR"
            print(f"\n[SUMMARY] [{status}] "
                f"PTOT={measured_kw:.2f} kW | SP={setpoint_kw:.1f} kW | "
                f"Err={error_kw:+.2f} kW | Ramped={ramped_kw:.2f} kW | "
                f"C_Pac={percent:.2f}%")

            time.sleep(LOOP_INTERVAL)

    except KeyboardInterrupt:
        print("\n\n[INFO] Stopped by user (KeyboardInterrupt).")
    finally:
        meter_client.close()
        inv_client.close()
        print("[INFO] Both connections closed.")


if __name__ == "__main__":
    def run_flask():
        app.run(host="0.0.0.0", port=5000)

    threading.Thread(target=run_flask, daemon=True).start()
    main()
