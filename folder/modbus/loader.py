import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from typing import Optional
from pathlib import Path

# Base directory of the project — always the parent of the modbus/ package
# This means paths work regardless of where you launch python from
_PROJECT_ROOT = Path(__file__).resolve().parent.parent


# ── Data models ────────────────────────────────────────────────────────────────

@dataclass
class RegisterDef:
    """A single read register definition."""
    name:   str
    hi_off: int          # index into flat poll buffer (high word)
    lo_off: Optional[int]  # index into flat poll buffer (low word), None for 16-bit
    scale:  float
    dtype:  str
    control: bool        # True = this register is relevant to control logic


@dataclass
class WriteRegisterDef:
    """A single write register definition."""
    name:  str
    addr:  int           # flat Modbus address to write to
    scale: float
    dtype: str


@dataclass
class DeviceConfig:
    """A read device (TCP or RTU)."""
    name:        str
    vendor:      str
    protocol:    str     # "TCP" or "RTU"
    host:        Optional[str]
    port:        Optional[int]
    device_id:   int
    blocks:      list    # [(start_addr, count), ...]
    registers:   list    # [RegisterDef, ...]
    addr_to_idx: dict    # {modbus_addr: flat_index}


@dataclass
class WriteDeviceConfig:
    """A write-only device — never polled, only used by control logic."""
    name:      str
    vendor:    str
    host:      str
    port:      int
    device_id: int
    registers: list      # [WriteRegisterDef, ...]


@dataclass
class RTUBusConfig:
    """One serial bus with one or more RTU devices on it."""
    name:     str
    port:     str
    baudrate: int
    parity:   str
    stopbits: int
    devices:  list       # [DeviceConfig, ...]


# ── Helpers ────────────────────────────────────────────────────────────────────

def _build_addr_index(blocks: list) -> dict:
    """
    Build a mapping of {modbus_address: flat_buffer_index} from block definitions.
    This is what lets decode_register work on a flat list instead of a dict.
    """
    idx = 0
    addr_to_idx = {}
    for start, count in blocks:
        for i in range(count):
            addr_to_idx[start + i] = idx
            idx += 1
    return addr_to_idx


def _parse_vendor_defs(registers_file: str) -> dict:
    """
    Parse registers.xml and return a dict keyed by vendor name.
    Each entry contains everything the loader needs to build DeviceConfig objects.

    Returns:
        {
            vendor_name: {
                "blocks":        [(start, count), ...],
                "registers":     [RegisterDef, ...],
                "addr_to_idx":   {addr: idx},
                "write_registers": [WriteRegisterDef, ...]
            }
        }
    """
    tree = ET.parse(registers_file)
    vendor_defs = {}

    for vendor in tree.getroot().findall("vendor"):
        vname = vendor.attrib["name"]

        # ── Read blocks ───────────────────────────────────────────────────────
        blocks = [
            (int(b.attrib["start"]), int(b.attrib["count"]))
            for b in vendor.find("read_blocks").findall("block")
        ]
        addr_to_idx = _build_addr_index(blocks)

        # ── Read registers ────────────────────────────────────────────────────
        registers = []
        for r in vendor.find("read_registers").findall("register"):
            hi_addr = int(r.attrib["hi"])
            lo_str  = r.attrib.get("lo", "").strip()
            lo_addr = int(lo_str) if lo_str else None
            control = r.attrib.get("control", "false").strip().lower() == "true"

            registers.append(RegisterDef(
                name    = r.attrib["name"],
                hi_off  = addr_to_idx[hi_addr],
                lo_off  = addr_to_idx[lo_addr] if lo_addr is not None else None,
                scale   = float(r.attrib["scale"]),
                dtype   = r.attrib["dtype"],
                control = control,
            ))

        # ── Write registers (optional — only some vendors have them) ──────────
        write_registers = []
        write_node = vendor.find("write_registers")
        if write_node is not None:
            for r in write_node.findall("register"):
                write_registers.append(WriteRegisterDef(
                    name  = r.attrib["name"],
                    addr  = int(r.attrib["addr"]),
                    scale = float(r.attrib["scale"]),
                    dtype = r.attrib["dtype"],
                ))

        vendor_defs[vname] = {
            "blocks":          blocks,
            "registers":       registers,
            "addr_to_idx":     addr_to_idx,
            "write_registers": write_registers,
        }

    return vendor_defs


# ── Main loader ────────────────────────────────────────────────────────────────

def load_all(
    devices_file:   str = "config/devices.xml",
    registers_file: str = "config/registers.xml",
):
    # Resolve relative paths from project root so they work from any working directory
    devices_file   = str(_PROJECT_ROOT / devices_file)
    registers_file = str(_PROJECT_ROOT / registers_file)
    """
    Parse both XML files and return all device configs separated by role.

    Returns:
        tcp_devices  : [DeviceConfig]       — TCP read devices
        rtu_buses    : [RTUBusConfig]        — RTU buses with read devices
        write_devices: [WriteDeviceConfig]   — write-only TCP devices
    """
    vendor_defs   = _parse_vendor_defs(registers_file)
    dev_tree      = ET.parse(devices_file)

    tcp_devices   = []
    rtu_buses     = []
    write_devices = []

    for vendor in dev_tree.getroot().findall("vendor"):
        vname = vendor.attrib["name"]
        vdef  = vendor_defs[vname]

        # ── TCP devices (read and write, separated by role attr) ──────────────
        devices_node = vendor.find("devices")
        if devices_node is not None:
            for d in devices_node.findall("device"):
                role     = d.attrib.get("role", "read").lower()
                protocol = d.find("protocol").text.strip().upper()

                if role == "write":
                    # Write devices get their own lightweight config
                    write_devices.append(WriteDeviceConfig(
                        name      = d.attrib["name"],
                        vendor    = vname,
                        host      = d.find("host").text.strip(),
                        port      = int(d.find("port").text),
                        device_id = int(d.find("device_id").text),
                        registers = vdef["write_registers"],
                    ))

                elif protocol == "TCP":
                    tcp_devices.append(DeviceConfig(
                        name        = d.attrib["name"],
                        vendor      = vname,
                        protocol    = "TCP",
                        host        = d.find("host").text.strip(),
                        port        = int(d.find("port").text),
                        device_id   = int(d.find("device_id").text),
                        blocks      = vdef["blocks"],
                        registers   = vdef["registers"],
                        addr_to_idx = vdef["addr_to_idx"],
                    ))

        # ── RTU buses (always read-only) ──────────────────────────────────────
        for bus in vendor.findall("bus"):
            bus_devices = []
            for d in bus.find("devices").findall("device"):
                bus_devices.append(DeviceConfig(
                    name        = d.attrib["name"],
                    vendor      = vname,
                    protocol    = "RTU",
                    host        = None,
                    port        = None,
                    device_id   = int(d.find("device_id").text),
                    blocks      = vdef["blocks"],
                    registers   = vdef["registers"],
                    addr_to_idx = vdef["addr_to_idx"],
                ))

            rtu_buses.append(RTUBusConfig(
                name     = bus.attrib["name"],
                port     = bus.find("port").text.strip(),
                baudrate = int(bus.find("baudrate").text),
                parity   = bus.find("parity").text.strip(),
                stopbits = int(bus.find("stopbits").text),
                devices  = bus_devices,
            ))

    return tcp_devices, rtu_buses, write_devices
