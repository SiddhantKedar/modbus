from pymodbus.client import ModbusTcpClient

from modbus.decoder import encode_write_register


# ── Device Writer ──────────────────────────────────────────────────────────────

class DeviceWriter:
    """
    One persistent TCP connection for a write-only device.
    Never polls — only writes single registers on demand.
    Reconnects automatically on failure.
    """

    def __init__(self, config):
        self.config  = config
        self._client = None
        # Build a quick lookup so callers can write by register name
        self._reg_map = {r.name: r for r in config.registers}

    def _ensure_connected(self) -> bool:
        if self._client and self._client.is_socket_open():
            return True
        self._client = ModbusTcpClient(self.config.host, port=self.config.port)
        connected = self._client.connect()
        if not connected:
            print(f"[WRITER] Failed to connect to {self.config.name} "
                  f"({self.config.host}:{self.config.port})")
        return connected

    def write(self, register_name: str, value: float) -> bool:
        """
        Write a physical value to a named register.

        The value is encoded using the register's scale and dtype
        defined in registers.xml — no magic numbers in the caller.

        Example:
            writer.write("c_pac", 75.0)
            → raw = 75.0 / 0.01 = 7500 → sent as u16 word to addr 40023

        Returns True on success, False on any failure.
        """
        reg = self._reg_map.get(register_name)
        if reg is None:
            print(f"[WRITER] Unknown register '{register_name}' "
                  f"for device '{self.config.name}'")
            return False

        if not self._ensure_connected():
            return False

        try:
            word = encode_write_register(value, reg.scale, reg.dtype)

            result = self._client.write_register(
                address   = reg.addr,
                value     = word,
                device_id = self.config.device_id,
            )

            if result.isError():
                print(f"[WRITER] Error writing '{register_name}'={value} "
                      f"(raw={word}) to {self.config.name}: {result}")
                self._client.close()
                self._client = None
                return False

            print(f"[WRITER] {self.config.name} '{register_name}'={value} "
                  f"→ raw={word} addr={reg.addr} OK")
            return True

        except Exception as e:
            print(f"[WRITER] Exception writing '{register_name}' "
                  f"to {self.config.name}: {e}")
            self._client.close()
            self._client = None
            return False

    def close(self):
        if self._client:
            self._client.close()
            self._client = None
