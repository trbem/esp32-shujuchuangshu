import json
import sqlite3
import tempfile
import unittest
from pathlib import Path

from fastapi.testclient import TestClient
from server.app import SCHEMA, create_app


class ServerTests(unittest.TestCase):
    def setUp(self):
        # ignore_cleanup_errors: SQLite on Windows keeps the .sqlite3 file (and
        # its -wal/-shm siblings) locked for a moment after close, which would
        # otherwise turn a passing test into a teardown error.
        self.tmp = tempfile.TemporaryDirectory(ignore_cleanup_errors=True)
        self.db = Path(self.tmp.name) / "test.sqlite3"
        self.client = TestClient(create_app(self.db, "test-key"))
        self.payload = {"device_id": "s3eye-001", "samples": [{
            "boot_id": "0000000000000001", "sequence": 1, "captured_at_ms": 1700000000000,
            "temp_c": 25.5, "heap_int": 1, "heap_psram": 2, "fps": 3, "jpeg": 4,
            "accel_ok": True, "accel_x_g": 0, "accel_y_g": 0, "accel_z_g": 1,
            "accel_mag_g": 1, "accel_range_g": 4, "accel_id": "0x90",
            "uptime_s": 5, "cpu_mhz": 240, "rssi": -40, "ip": "192.168.1.2"}]}

    def tearDown(self): self.tmp.cleanup()

    def query(self, sql: str) -> list[tuple]:
        """Open, read, close - `with sqlite3.connect(...)` commits but does NOT
        close, and a leaked handle blocks the temp directory cleanup."""
        conn = sqlite3.connect(self.db)
        try:
            return conn.execute(sql).fetchall()
        finally:
            conn.close()

    def test_auth_insert_and_idempotency(self):
        self.assertEqual(self.client.post('/api/v1/telemetry', json=self.payload).status_code, 401)
        headers = {"X-Api-Key": "test-key"}
        self.assertEqual(self.client.post('/api/v1/telemetry', json=self.payload, headers=headers).json()["inserted"], 1)
        self.assertEqual(self.client.post('/api/v1/telemetry', json=self.payload, headers=headers).json()["inserted"], 0)
        self.assertEqual(len(self.client.get('/api/v1/telemetry').json()["samples"]), 1)
        self.assertIn('temp_c', self.client.get('/api/v1/export.csv').text)
        self.assertIn('ESP32 遥测历史', self.client.get('/').text)

    def test_lcd_counters_are_optional_and_persisted(self):
        """Older firmware omits lcd_frames/lcd_fps; newer firmware sends them.
        Both must be accepted, and the values must survive into SQLite."""
        headers = {"X-Api-Key": "test-key"}

        # 1. a sample without the fields (older firmware) still inserts
        self.assertEqual(
            self.client.post('/api/v1/telemetry', json=self.payload, headers=headers).json()["inserted"], 1)

        # 2. a sample carrying them stores the real numbers
        payload = json.loads(json.dumps(self.payload))
        payload["samples"][0].update(sequence=2, lcd_frames=2187, lcd_fps=24.0, lcd_err=0)
        self.assertEqual(
            self.client.post('/api/v1/telemetry', json=payload, headers=headers).json()["inserted"], 1)

        # 3. and a failing viewfinder is recorded as such, not silently dropped
        payload["samples"][0].update(sequence=3, lcd_frames=2187, lcd_fps=0.0, lcd_err=41)
        self.assertEqual(
            self.client.post('/api/v1/telemetry', json=payload, headers=headers).json()["inserted"], 1)

        rows = {seq: (frames, err) for seq, frames, err in self.query(
            "SELECT sequence, lcd_frames, lcd_err FROM telemetry")}
        self.assertEqual(rows[1], (0, 0))       # omitted -> column defaults
        self.assertEqual(rows[2], (2187, 0))    # sent -> persisted
        self.assertEqual(rows[3], (2187, 41))   # failures survive into history

    def test_columns_added_to_a_pre_existing_table(self):
        """A database created before lcd_frames existed must be migrated on open.

        The legacy schema is derived from the current SCHEMA with the two new
        columns stripped out, so the test keeps matching reality if the table
        grows more columns later.
        """
        legacy = Path(self.tmp.name) / "legacy.sqlite3"
        legacy_schema = "\n".join(
            line for line in SCHEMA.splitlines()
            if "lcd_frames" not in line)
        conn = sqlite3.connect(legacy)
        try:
            conn.executescript(legacy_schema)
            conn.commit()
        finally:
            conn.close()

        create_app(legacy, "test-key")      # opening it must run the migration

        conn = sqlite3.connect(legacy)
        try:
            cols = {r[1] for r in conn.execute("PRAGMA table_info(telemetry)")}
        finally:
            conn.close()
        self.assertIn("lcd_frames", cols)
        self.assertIn("lcd_fps", cols)
        self.assertIn("lcd_err", cols)


if __name__ == '__main__': unittest.main()
