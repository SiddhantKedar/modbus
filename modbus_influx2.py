from pymodbus.client import ModbusTcpClient, ModbusSerialClient
import struct
from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
from datetime import datetime, timezone
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Optional
from concurrent.futures import ThreadPoolExecutor, wait, as_completed
import serial.tools.list_ports

INFLUX_URL    = "http://localhost:8086"
INFLUX_TOKEN  = "hbpc65tI5EpD_nR8mTcMrC1naDetwSjzwmPrhtCYAYrzmUIroOZ5SuZ7Acyl9rSTz2yNDOZUSEcKHwl5B2LxOw=="
INFLUX_ORG    = "PiTest"
INFLUX_BUCKET = "modbus_float"
MONITOR_INTERVAL = 10   # seconds
CONTROL_INTERVAL = 1
RTU_TIMEOUT   = 0.3   # seconds — tune per device

SENTINEL_U16 = 0xFFFF
SENTINEL_U32 = 0xFFFFFFFF

MAX_WORKERS = 5


# ── Data model ────────────────────────────────────────────────────────────────

@dataclass
class RegisterDef:
    name:   str
    hi_off: int
    lo_off: Optional[int]
    scale:  float
    dtype:  str
    control: bool

@dataclass
class DeviceConfig:
    name:       str
    host:       str
    port:       int
    device_id:  int
    vendor:     str
    protocol:   str
    blocks:     list
    registers:  list
    addr_to_idx: dict = field(default_factory=dict)

@dataclass
class RTUBusConfig:
    name:     str
    port:     str
    baudrate: int
    parity:   str
    stopbits: int
    devices:  list


# ── XML parsing ───────────────────────────────────────────────────────────────

def build_addr_index(blocks):
    idx = 0
    addr_to_idx = {}
    for start, count in blocks:
        for i in range(count):
            addr_to_idx[start + i] = idx
            idx += 1
    return addr_to_idx

def load_all(devices_file="devices.xml", registers_file="registers.xml"):
    reg_tree = ET.parse(registers_file)
    vendor_defs = {}

    for vendor in reg_tree.getroot().findall("vendor"):
        vname  = vendor.attrib["name"]
        blocks = [(int(b.attrib["start"]), int(b.attrib["count"]))
                  for b in vendor.find("read_blocks").findall("block")]
        addr_to_idx = build_addr_index(blocks)

        registers = []
        for r in vendor.find("read_registers").findall("register"):
            hi_addr = int(r.attrib["hi"])
            lo_str  = r.attrib.get("lo", "")
            lo_addr = int(lo_str) if lo_str else None
            control = r.attrib.get("control", "false").lower() == "true"
            registers.append(RegisterDef(
                name   = r.attrib["name"],
                hi_off = addr_to_idx[hi_addr],
                lo_off = addr_to_idx[lo_addr] if lo_addr is not None else None,
                scale  = float(r.attrib["scale"]),
                dtype  = r.attrib["dtype"],
                control = control,
            ))

        vendor_defs[vname] = (blocks, registers, addr_to_idx)

    dev_tree    = ET.parse(devices_file)
    tcp_devices = []
    rtu_buses   = []

    for vendor in dev_tree.getroot().findall("vendor"):
        vname  = vendor.attrib["name"]
        blocks, registers, addr_to_idx = vendor_defs[vname]

        devices_node = vendor.find("devices")
        if devices_node is not None:
            for d in devices_node.findall("device"):
                if d.find("protocol").text.upper() == "TCP":
                    tcp_devices.append(DeviceConfig(
                        name        = d.attrib["name"],
                        host        = d.find("host").text,
                        port        = int(d.find("port").text),
                        device_id   = int(d.find("device_id").text),
                        vendor      = vname,
                        protocol    = "TCP",
                        blocks      = blocks,
                        registers   = registers,
                        addr_to_idx = addr_to_idx,
                    ))

        for bus in vendor.findall("bus"):
            bus_devices = []
            for d in bus.find("devices").findall("device"):
                bus_devices.append(DeviceConfig(
                    name        = d.attrib["name"],
                    host        = None,
                    port        = None,
                    device_id   = int(d.find("device_id").text),
                    vendor      = vname,
                    protocol    = "RTU",
                    blocks      = blocks,
                    registers   = registers,
                    addr_to_idx = addr_to_idx,
                ))
            rtu_buses.append(RTUBusConfig(
                name     = bus.attrib["name"],
                port     = bus.find("port").text,
                baudrate = int(bus.find("baudrate").text),
                parity   = bus.find("parity").text,
                stopbits = int(bus.find("stopbits").text),
                devices  = bus_devices,
            ))

    return tcp_devices, rtu_buses


# ── Decoding ──────────────────────────────────────────────────────────────────

def to_s32(v):
    return v - 0x100000000 if v & 0x80000000 else v

def decode_register(flat, reg):
    hi = flat[reg.hi_off]

    if reg.lo_off is None:                      # 16-bit
        if hi == SENTINEL_U16:
            return None
        if reg.dtype == "S16":
            hi = hi - 0x10000 if hi & 0x8000 else hi
        return round(hi * reg.scale, 2)

    lo  = flat[reg.lo_off]                      # 32-bit
    raw = (hi << 16) | lo

    if raw == SENTINEL_U32:
        return None

    if reg.dtype == "F32":                      # ← fixed: early return, correct byte order
        raw_bytes = struct.pack('>HH', lo, hi)
        return round(struct.unpack('>f', raw_bytes)[0], 2)

    if reg.dtype == "S32":
        raw = to_s32(raw)

    return round(raw * reg.scale)

# --- Port fidning for RTU


def find_rtu_port(bus_config: RTUBusConfig, probe_device_id: int, probe_register: int = 0, timeout: float = 0.5) -> Optional[str]:
    """
    Scan all available serial ports and return the one that responds
    to a Modbus RTU request from the given device_id.
    """
    available_ports = [p.device for p in serial.tools.list_ports.comports()]
    print(f"[RTU] Scanning ports: {available_ports}")

    for port in available_ports:
        try:
            client = ModbusSerialClient(
                port     = port,
                baudrate = bus_config.baudrate,
                parity   = bus_config.parity,
                stopbits = bus_config.stopbits,
                timeout  = timeout,
            )

            if not client.connect():
                continue

            result = client.read_holding_registers(
                address   = probe_register,
                count     = 1,
                device_id = probe_device_id
            )
            client.close()

            if not result.isError():
                print(f"[RTU] Found '{bus_config.name}' on {port}")
                return port

        except Exception as e:
            print(f"[RTU] Port {port} failed: {e}")
            continue

    print(f"[RTU] Could not find '{bus_config.name}' on any port")
    return None

# ── Pollers ───────────────────────────────────────────────────────────────────

class DevicePoller:
    """One persistent TCP connection per device."""
    def __init__(self, config):
        self.config  = config
        self._client = None

    def _ensure_connected(self):
        if self._client and self._client.is_socket_open():
            return True
        self._client = ModbusTcpClient(self.config.host, port=self.config.port)
        return self._client.connect()

    def poll(self):
        if not self._ensure_connected():
            return self.config.name, None

        flat = []
        for start, count in self.config.blocks:
            result = self._client.read_holding_registers(
                address=start, count=count, device_id=self.config.device_id
            )
            if result.isError():
                self._client.close()
                self._client = None
                return self.config.name, None
            flat.extend(result.registers)

        data = {reg.name: decode_register(flat, reg) for reg in self.config.registers}
        return self.config.name, data

    def close(self):
        if self._client:
            self._client.close()
            self._client = None


class RTUBusPoller:
    """One persistent serial connection per bus; devices polled sequentially."""
    def __init__(self, bus_config):
        self.bus    = bus_config
        self.client = None
        self._port = bus_config.port

    def _ensure_connected(self):
        if self.client:
            return True
        self.client = ModbusSerialClient(
            port     = self._port,
            baudrate = self.bus.baudrate,
            parity   = self.bus.parity,
            stopbits = self.bus.stopbits,
            timeout  = RTU_TIMEOUT,          # tunable, not hardcoded 1s
        )
        if self.client.connect():
            return True
        
        
        print(f"[RTU] '{self.bus.name}' not on {self._port}, scanning...")
        self.client = None

        probe_device = self.bus.devices[0]   # use first device to probe
        found_port   = find_rtu_port(
            bus_config      = self.bus,
            probe_device_id = probe_device.device_id,
            probe_register  = probe_device.blocks[0][0],   # first register of first block
        )

        if found_port:
            self._port = found_port          # remember it for next reconnect
            self.client = ModbusSerialClient(
                port     = self._port,
                baudrate = self.bus.baudrate,
                parity   = self.bus.parity,
                stopbits = self.bus.stopbits,
                timeout  = RTU_TIMEOUT,
            )
            return self.client.connect()

        self.client = None
        return False

    def poll(self):
        """Returns list of (device_name, data_dict). Failed devices are skipped,
        not returned as None — so the caller never writes a bad point."""
        if not self._ensure_connected():
            return []

        results = []

        for device in self.bus.devices:
            flat = []
            failed = False

            for start, count in device.blocks:
                try:
                    result = self.client.read_holding_registers(
                        address=start, count=count, device_id=device.device_id
                    )
                    if result.isError():
                        print(f"[RTU] Read error: {device.name} block {start}")
                        failed = True
                        break                # skip remaining blocks for THIS device only
                    flat.extend(result.registers)

                except Exception as e:
                    print(f"[RTU] Exception on {device.name}: {e}")
                    self.client.close()
                    self.client = None
                    return results           # return whatever succeeded so far

            if not failed and flat:
                data = {reg.name: decode_register(flat, reg) for reg in device.registers}
                results.append((device.name, data))

        return results

    def close(self):
        if self.client:
            self.client.close()
            self.client = None

# Control Logic
def run_control_logic(results):

    device_map = {name: data for name, data, _ in results}

    inv = device_map.get("inv1", {})
    meter = device_map.get("meter1", {})

    active_power = inv.get("active_power")
    qac          = inv.get("qac")
    ptot         = meter.get("Ptot")

    print(f"[CONTROL] AP={active_power}, QAC={qac}, PTOT={ptot}")

# ── InfluxDB ──────────────────────────────────────────────────────────────────

def build_point(data: dict, device_name: str, timestamp: datetime) -> Point:
    point = Point("solar_data").tag("inverter", device_name)
    point.time(timestamp)
    for key, value in data.items():
        if value is not None:
            point.field(key, value)
    return point
# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    tcp_devices, rtu_buses = load_all()
    tcp_pollers = [DevicePoller(d)  for d in tcp_devices]
    rtu_pollers = [RTUBusPoller(b)  for b in rtu_buses]
    all_pollers = tcp_pollers + rtu_pollers

    influx    = InfluxDBClient(url=INFLUX_URL, token=INFLUX_TOKEN, org=INFLUX_ORG)
    write_api = influx.write_api(write_options=SYNCHRONOUS)

    # Single persistent executor
    executor = ThreadPoolExecutor(max_workers=MAX_WORKERS)

    last_monitor = time.monotonic()   # ← fix issue 3

    try:
        while True:
            loop_start = time.monotonic()
            poll_ts = datetime.now(timezone.utc)   # ← snapshot before poll

            # --- 1. POLL (parallel, persistent executor) ---
            futures = {executor.submit(p.poll): p for p in all_pollers}
            results = []   # [(name, data, timestamp), ...]

            for future in as_completed(futures):
                result = future.result()

                if isinstance(result, tuple):          # TCP
                    name, data = result
                    if data:
                        results.append((name, data, poll_ts))

                elif isinstance(result, list):         # RTU
                    for name, data in result:
                        results.append((name, data, poll_ts))

            # --- 2. CONTROL (every 1s, parallel) ---
            # Run control logic concurrently, don't block main thread
            control_futures = [
                executor.submit(run_control_logic, results)
            ]
            # Don't wait — fire and move on, control runs in background
            # If control must complete before next poll, add:
            # wait(control_futures, timeout=0.5)

            # --- 3. MONITOR (every 10s) ---
            now = time.monotonic()
            if now - last_monitor >= MONITOR_INTERVAL:

                # Collect all points, single write call
                points = [
                    build_point(data, name, ts)
                    for name, data, ts in results
                ]

                if points:
                    write_api.write(bucket=INFLUX_BUCKET, record=points)

                last_monitor = now
                print(f"[MONITOR] Wrote {len(points)} points")
                print({name: data for name, data, _ in results})

            # --- 4. TIMING ---
            elapsed    = time.monotonic() - loop_start
            sleep_time = max(0, CONTROL_INTERVAL - elapsed)
            time.sleep(sleep_time)

    finally:
        executor.shutdown(wait=False)
        for p in tcp_pollers:
            p.close()
        for p in rtu_pollers:
            p.close()
        influx.close()

if __name__ == "__main__":
    main()
