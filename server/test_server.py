import json
import sqlite3
import struct
import tempfile
import unittest
from pathlib import Path

from fastapi.testclient import TestClient
from server.app import PHOTO_BYTE_SIZE, PHOTO_HEADER_SIZE, PHOTO_HEIGHT, PHOTO_WIDTH, SCHEMA, create_app


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

    @staticmethod
    def photo_bmp() -> bytes:
        """The exact 240x240 top-down BMP the ESP32 uploads."""
        bmp = bytearray(PHOTO_BYTE_SIZE)
        bmp[:2] = b"BM"
        struct.pack_into("<I", bmp, 2, PHOTO_BYTE_SIZE)
        struct.pack_into("<I", bmp, 10, PHOTO_HEADER_SIZE)
        struct.pack_into("<I", bmp, 14, 40)
        struct.pack_into("<i", bmp, 18, PHOTO_WIDTH)
        struct.pack_into("<i", bmp, 22, -PHOTO_HEIGHT)
        struct.pack_into("<H", bmp, 26, 1)
        struct.pack_into("<H", bmp, 28, 24)
        struct.pack_into("<I", bmp, 34, PHOTO_WIDTH * PHOTO_HEIGHT * 3)
        return bytes(bmp)

    def test_auth_insert_and_idempotency(self):
        self.assertEqual(self.client.post('/api/v1/telemetry', json=self.payload).status_code, 401)
        headers = {"X-Api-Key": "test-key"}
        self.assertEqual(self.client.post('/api/v1/telemetry', json=self.payload, headers=headers).json()["inserted"], 1)
        self.assertEqual(self.client.post('/api/v1/telemetry', json=self.payload, headers=headers).json()["inserted"], 0)
        self.assertEqual(len(self.client.get('/api/v1/telemetry').json()["samples"]), 1)
        self.assertIn('temp_c', self.client.get('/api/v1/export.csv').text)
        self.assertIn('<title>ESP32', self.client.get('/').text)

    def test_live_telemetry_is_authenticated_current_and_not_historical(self):
        headers = {"X-Api-Key": "test-key"}
        self.assertEqual(self.client.post("/api/v1/live-telemetry", json=self.payload).status_code, 401)
        received = self.client.post("/api/v1/live-telemetry", json=self.payload, headers=headers)
        self.assertEqual(received.status_code, 200)
        live = self.client.get("/api/v1/live-telemetry?device_id=s3eye-001").json()["sample"]
        self.assertEqual((live["device_id"], live["temp_c"], live["source"]), ("s3eye-001", 25.5, "live"))
        self.assertIn("received_at_ms", live)
        self.assertEqual(len(self.client.get('/api/v1/telemetry').json()["samples"]), 0)

        newer = json.loads(json.dumps(self.payload))
        newer["samples"][0].update(sequence=2, temp_c=26.5)
        self.client.post("/api/v1/live-telemetry", json=newer, headers=headers)
        self.assertEqual(self.client.get("/api/v1/live-telemetry?device_id=s3eye-001").json()["sample"]["temp_c"], 26.5)

    def test_remote_ingest_host_only_exposes_authenticated_device_endpoints(self):
        client = TestClient(create_app(self.db, "test-key", dashboard_host="dashboard.example.test",
                                       ingest_host="ingest.example.test"))
        device_headers = {"Host": "ingest.example.test", "X-Api-Key": "test-key"}
        self.assertEqual(client.post("/api/v1/telemetry", json=self.payload, headers=device_headers).status_code, 200)
        self.assertEqual(client.get("/api/v1/telemetry", headers={"Host": "ingest.example.test"}).status_code, 404)
        self.assertEqual(client.post("/api/v1/capture-tasks", json={"device_id": "s3eye-001"},
                                     headers={"Host": "ingest.example.test"}).status_code, 404)
        photo_request = "/api/v1/devices/s3eye-001/capture-photo"
        self.assertEqual(client.post(photo_request, headers={"Host": "ingest.example.test"}).status_code, 401)
        created = client.post(photo_request, headers=device_headers)
        self.assertEqual((created.status_code, created.json()["task_type"]), (201, "capture_photo"))
        self.assertEqual(client.get("/", headers={"Host": "dashboard.example.test"}).status_code, 200)
        self.assertEqual(client.get("/", headers={"Host": "unknown.example.test"}).status_code, 421)

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
        legacy_schema = legacy_schema.replace(
            "error TEXT, telemetry_id INTEGER, photo_id INTEGER,\n"
            "  FOREIGN KEY(telemetry_id) REFERENCES telemetry(id),\n"
            "  FOREIGN KEY(photo_id) REFERENCES photos(id)",
            "error TEXT, telemetry_id INTEGER,\n"
            "  FOREIGN KEY(telemetry_id) REFERENCES telemetry(id)",
        )
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
            task_cols = {r[1] for r in conn.execute("PRAGMA table_info(capture_tasks)")}
        finally:
            conn.close()
        self.assertIn("lcd_frames", cols)
        self.assertIn("lcd_fps", cols)
        self.assertIn("lcd_err", cols)
        self.assertIn("photo_id", task_cols)

    def test_on_demand_task_ack_result_and_periodic_pause(self):
        headers = {"X-Api-Key": "test-key"}
        # Repeated clicks intentionally create distinct tasks.
        first = self.client.post("/api/v1/capture-tasks", json={"device_id": "s3eye-001"}).json()
        second = self.client.post("/api/v1/capture-tasks", json={"device_id": "s3eye-001"}).json()
        self.assertNotEqual(first["request_id"], second["request_id"])
        next_task = self.client.get("/api/v1/devices/s3eye-001/tasks/next", headers=headers).json()["task"]
        self.assertEqual(next_task["request_id"], first["request_id"])
        self.assertEqual(self.client.post(f"/api/v1/capture-tasks/{first['request_id']}/ack",
                                          headers=headers, json={"device_id": "s3eye-001"}).json()["status"], "received")
        result = {"device_id": "s3eye-001", "sample": dict(self.payload["samples"][0])}
        result["sample"]["sequence"] = 99
        finished = self.client.post(f"/api/v1/capture-tasks/{first['request_id']}/result",
                                    headers=headers, json=result).json()
        self.assertEqual(finished["status"], "completed")
        self.assertIsNotNone(finished["telemetry_id"])
        self.assertEqual(self.query("SELECT source, request_id FROM telemetry WHERE id=%d" % finished["telemetry_id"])[0],
                         ("on_demand", first["request_id"]))
        self.assertEqual(self.client.put("/api/v1/control/periodic-ingestion", json={"paused": True}).json(), {"paused": True})
        self.assertEqual(self.client.post('/api/v1/telemetry', json=self.payload, headers=headers).json()["inserted"], 0)

    def test_unreceived_task_times_out(self):
        task = self.client.post("/api/v1/capture-tasks", json={"device_id": "offline-device"}).json()
        conn = sqlite3.connect(self.db)
        try:
            conn.execute("UPDATE capture_tasks SET expires_at_ms=0 WHERE request_id=?", (task["request_id"],))
            conn.commit()
        finally:
            conn.close()
        tasks = self.client.get("/api/v1/capture-tasks").json()["tasks"]
        expired = next(t for t in tasks if t["request_id"] == task["request_id"])
        self.assertEqual(expired["status"], "timeout")

    def test_photo_task_upload_is_authenticated_idempotent_and_retained(self):
        device_headers = {"X-Api-Key": "test-key"}
        headers = {**device_headers, "X-Device-Id": "s3eye-001", "X-Captured-At-Ms": "1700000000123",
                   "Content-Type": "image/bmp"}
        task = self.client.post("/api/v1/capture-tasks", json={
            "device_id": "s3eye-001", "task_type": "capture_photo"}).json()
        self.assertEqual(task["task_type"], "capture_photo")
        self.assertEqual(self.client.get("/api/v1/devices/s3eye-001/tasks/next", headers=device_headers).json()["task"]["request_id"],
                         task["request_id"])
        self.assertEqual(self.client.post(f"/api/v1/capture-tasks/{task['request_id']}/ack", headers=device_headers,
                                          json={"device_id": "s3eye-001"}).json()["status"], "received")

        # No device key -> no photo may be stored.
        self.assertEqual(self.client.post(f"/api/v1/capture-tasks/{task['request_id']}/photo", content=self.photo_bmp(),
                                          headers={"X-Device-Id": "s3eye-001", "X-Captured-At-Ms": "1700000000123",
                                                   "Content-Type": "image/bmp"}).status_code, 401)
        # A bad body is rejected even with an authenticated device.
        self.assertEqual(self.client.post(f"/api/v1/capture-tasks/{task['request_id']}/photo", content=b"bad",
                                          headers=headers).status_code, 409)

        finished = self.client.post(f"/api/v1/capture-tasks/{task['request_id']}/photo", content=self.photo_bmp(),
                                    headers=headers).json()
        self.assertEqual(finished["status"], "completed")
        self.assertIsNotNone(finished["photo_id"])
        self.assertEqual(self.query("SELECT count(*) FROM photos")[0][0], 1)
        image = self.client.get(f"/api/v1/photos/{finished['photo_id']}/image")
        self.assertEqual((image.status_code, image.headers["content-type"], len(image.content)),
                         (200, "image/bmp", PHOTO_BYTE_SIZE))

        # Retrying the same completed task confirms the existing photo rather than writing another file/row.
        retried = self.client.post(f"/api/v1/capture-tasks/{task['request_id']}/photo", content=self.photo_bmp(),
                                   headers=headers).json()
        self.assertEqual((retried["status"], retried["photo_id"]), ("completed", finished["photo_id"]))
        self.assertEqual(self.query("SELECT count(*) FROM photos")[0][0], 1)

        conn = sqlite3.connect(self.db)
        try:
            conn.execute("UPDATE photos SET received_at_ms=0 WHERE id=?", (finished["photo_id"],))
            conn.commit()
        finally:
            conn.close()
        self.client.app.state.store.cleanup(force=True)
        self.assertEqual(self.query("SELECT count(*) FROM photos")[0][0], 0)
        self.assertEqual(self.query("SELECT photo_id FROM capture_tasks WHERE request_id='%s'" % task["request_id"])[0][0], None)

    def test_device_events_are_authenticated_idempotent_and_trace_viewer_feedback(self):
        headers = {"X-Api-Key": "test-key"}
        event = {
            "device_id": "s3eye-001", "event_id": "9e2c1fa0-6648-4b5a-93df-0d343de0c19c",
            "event_type": "help_request", "occurred_at_ms": 1700000000999,
        }
        self.assertEqual(self.client.post("/api/v1/device-events", json=event).status_code, 401)
        created = self.client.post("/api/v1/device-events", json=event, headers=headers)
        self.assertEqual((created.status_code, created.json()["status"]), (201, "received"))
        replayed = self.client.post("/api/v1/device-events", json=event, headers=headers)
        self.assertEqual(replayed.json()["id"], created.json()["id"])
        self.assertEqual(self.query("SELECT count(*) FROM device_events")[0][0], 1)

        event_id = event["event_id"]
        device = self.client.get(f"/api/v1/device-events/{event_id}/device?device_id=s3eye-001", headers=headers)
        self.assertEqual(device.json()["status"], "received")
        replied = self.client.put(f"/api/v1/device-events/{event_id}/respond", json={"message": "已收到，请继续"})
        self.assertEqual((replied.status_code, replied.json()["status"], replied.json()["viewer_message"]),
                         (200, "responded", "已收到，请继续"))
        delivered = self.client.post(f"/api/v1/device-events/{event_id}/delivery-ack",
                                     headers=headers, json={"device_id": "s3eye-001"})
        self.assertIsNotNone(delivered.json()["device_acknowledged_at_ms"])
        self.assertEqual(self.client.post(f"/api/v1/device-events/{event_id}/cancel", headers=headers,
                                          json={"device_id": "s3eye-001"}).status_code, 409)

        cancelled = dict(event, event_id="7f19842b-d743-4bfe-b5e9-b4a755be4687", event_type="test_message")
        self.client.post("/api/v1/device-events", json=cancelled, headers=headers)
        remote_cancel = self.client.put(f"/api/v1/device-events/{cancelled['event_id']}/cancel")
        self.assertEqual((remote_cancel.status_code, remote_cancel.json()["status"], remote_cancel.json()["cancelled_by"]),
                         (200, "cancelled", "viewer"))
        self.assertIsNotNone(self.client.post(f"/api/v1/device-events/{cancelled['event_id']}/delivery-ack",
                                               headers=headers, json={"device_id": "s3eye-001"}).json()["device_acknowledged_at_ms"])

        conn = sqlite3.connect(self.db)
        try:
            conn.execute("UPDATE device_events SET occurred_at_ms=0, received_at_ms=0")
            conn.commit()
        finally:
            conn.close()
        self.client.app.state.store.cleanup(force=True)
        self.assertEqual(self.query("SELECT count(*) FROM device_events")[0][0], 0)


if __name__ == '__main__': unittest.main()
