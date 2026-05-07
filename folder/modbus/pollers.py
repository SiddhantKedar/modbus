from pymodbus.client import ModbusTcpClient, ModbusSerialClient
from typing import Optional
import serial.tools.list_ports

from modbus.decoder import decode_register

RTU_TIMEOUT = 0.3   # seconds — tune per device


# ── TCP Device Poller ──────────────────────────────────────────────────────────

class DevicePoller:
    """
    One persistent TCP connection per read device.
    Reconnects automatically on failure.
    """

    def __init__(self, config):
        self.config  = config
        self._client = None

    def _ensure_connected(self) -> bool:
        if self._client and self._client.is_socket_open():
            return True
        self._client = ModbusTcpClient(self.config.host, port=self.config.port)
        connected = self._client.connect()
        if not connected:
            print(f"[TCP] Failed to connect to {self.config.name} "
                  f"({self.config.host}:{self.config.port})")
        return connected

    def poll(self):
        """
        Read all blocks for this device and decode all registers.

        Returns:
            (device_name, data_dict)  on success
            (device_name, None)       on any failure
        """
        if not self._ensure_connected():
            return self.config.name, None

        flat = []
        for start, count in self.config.blocks:
            result = self._client.read_holding_registers(
                address=start, count=count, device_id=self.config.device_id
            )
            if result.isError():
                print(f"[TCP] Read error on {self.config.name} block start={start}: {result}")
                self._client.close()
                self._client = None
                return self.config.name, None
            flat.extend(result.registers)

        data = {
            reg.name: decode_register(flat, reg.hi_off, reg.lo_off, reg.scale, reg.dtype)
            for reg in self.config.registers
        }
        return self.config.name, data

    def close(self):
        if self._client:
            self._client.close()
            self._client = None


# ── RTU Port Scanner ───────────────────────────────────────────────────────────

def _find_rtu_port(
    bus_config,
    probe_device_id: int,
    probe_register:  int,
    timeout:         float = 0.5,
) -> Optional[str]:
    """
    Scan all available serial ports and return the first one that responds
    to a Modbus RTU request from the given device_id.
    Used as a fallback when the configured port is unavailable.
    """
    available = [p.device for p in serial.tools.list_ports.comports()]
    print(f"[RTU] Scanning ports for '{bus_config.name}': {available}")

    for port in available:
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
                device_id = probe_device_id,
            )
            client.close()

            if not result.isError():
                print(f"[RTU] Found '{bus_config.name}' on {port}")
                return port

        except Exception as e:
            print(f"[RTU] Port {port} error: {e}")
            continue

    print(f"[RTU] Could not find '{bus_config.name}' on any port")
    return None


# ── RTU Bus Poller ─────────────────────────────────────────────────────────────

class RTUBusPoller:
    """
    One persistent serial connection per bus.
    Devices on the same bus are polled sequentially.
    Falls back to port scanning if the configured port is unavailable.
    """

    def __init__(self, bus_config):
        self.bus    = bus_config
        self.client = None
        self._port  = bus_config.port   # may be updated by port scanner

    def _ensure_connected(self) -> bool:
        if self.client:
            return True

        self.client = ModbusSerialClient(
            port     = self._port,
            baudrate = self.bus.baudrate,
            parity   = self.bus.parity,
            stopbits = self.bus.stopbits,
            timeout  = RTU_TIMEOUT,
        )

        if self.client.connect():
            return True

        # Configured port failed — scan all available ports
        print(f"[RTU] '{self.bus.name}' not on {self._port}, scanning...")
        self.client = None

        probe_device = self.bus.devices[0]
        found_port   = _find_rtu_port(
            bus_config      = self.bus,
            probe_device_id = probe_device.device_id,
            probe_register  = probe_device.blocks[0][0],
        )

        if found_port:
            self._port  = found_port
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
        """
        Poll all devices on this bus sequentially.

        Returns a list of (device_name, data_dict) for every device that
        responded successfully. Failed devices are skipped — never returned
        as None — so the caller never writes a bad point to InfluxDB.
        """
        if not self._ensure_connected():
            return []

        results = []

        for device in self.bus.devices:
            flat   = []
            failed = False

            for start, count in device.blocks:
                try:
                    result = self.client.read_holding_registers(
                        address=start, count=count, device_id=device.device_id
                    )
                    if result.isError():
                        print(f"[RTU] Read error: {device.name} block start={start}")
                        failed = True
                        break          # skip remaining blocks for this device only

                    flat.extend(result.registers)

                except Exception as e:
                    print(f"[RTU] Exception on {device.name}: {e}")
                    self.client.close()
                    self.client = None
                    return results     # return whatever succeeded before the exception

            if not failed and flat:
                data = {
                    reg.name: decode_register(flat, reg.hi_off, reg.lo_off, reg.scale, reg.dtype)
                    for reg in device.registers
                }
                results.append((device.name, data))

        return results

    def close(self):
        if self.client:
            self.client.close()
            self.client = None
