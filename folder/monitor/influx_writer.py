from influxdb_client import InfluxDBClient, Point
from influxdb_client.client.write_api import SYNCHRONOUS
from datetime import datetime


# ── Constants ──────────────────────────────────────────────────────────────────

INFLUX_URL    = "http://localhost:8086"
INFLUX_TOKEN  = "hbpc65tI5EpD_nR8mTcMrC1naDetwSjzwmPrhtCYAYrzmUIroOZ5SuZ7Acyl9rSTz2yNDOZUSEcKHwl5B2LxOw=="
INFLUX_ORG    = "PiTest"
INFLUX_BUCKET = "modbus_live"


# ── InfluxDB client wrapper ────────────────────────────────────────────────────

class InfluxWriter:
    """
    Thin wrapper around the InfluxDB client.
    Holds one persistent connection and exposes a single write() call
    that takes the results list directly from the main loop.
    """

    def __init__(self):
        self._client    = InfluxDBClient(
            url   = INFLUX_URL,
            token = INFLUX_TOKEN,
            org   = INFLUX_ORG,
        )
        self._write_api = self._client.write_api(write_options=SYNCHRONOUS)

    def write(self, results: list, timestamp: datetime):
        """
        Build one Point per device and write all in a single batch.

        Args:
            results   : [(device_name, data_dict, poll_timestamp), ...]
                        as produced by the main loop
            timestamp : snapshot timestamp taken before the poll started
        """
        points = []
        for name, data, _ in results:
            point = Point("solar_data").tag("inverter", name).time(timestamp)
            for key, value in data.items():
                if value is not None:
                    point.field(key, float(value))
            points.append(point)

        if not points:
            print("[MONITOR] No points to write")
            return

        try:
            self._write_api.write(bucket=INFLUX_BUCKET, record=points)
            print(f"[MONITOR] Wrote {len(points)} points — "
                  f"devices: {[n for n, _, _ in results]}")
        except Exception as e:
            print(f"[MONITOR] InfluxDB write error: {e}")

    def close(self):
        self._client.close()
