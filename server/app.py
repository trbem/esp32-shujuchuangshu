"""LAN telemetry collector for the ESP32-S3-EYE dashboard."""
from __future__ import annotations

import csv
import hashlib
import io
import os
import sqlite3
import threading
import time
import uuid
from contextlib import contextmanager
from pathlib import Path
from typing import Literal

from fastapi import Depends, FastAPI, Header, HTTPException, Query, Request
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

ROOT = Path(__file__).resolve().parent


def load_dotenv(path: Path) -> None:
    """Tiny dependency-free .env reader; environment variables win."""
    if not path.exists():
        return
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, value = line.split("=", 1)
        os.environ.setdefault(key.strip(), value.strip().strip('"'))


load_dotenv(ROOT / ".env")


class TelemetrySample(BaseModel):
    boot_id: str = Field(min_length=1, max_length=32)
    sequence: int = Field(ge=1)
    captured_at_ms: int = Field(ge=0)
    temp_c: float
    heap_int: int = Field(ge=0)
    heap_psram: int = Field(ge=0)
    fps: float = Field(ge=0)
    jpeg: int = Field(ge=0)
    accel_ok: bool
    accel_x_g: float
    accel_y_g: float
    accel_z_g: float
    accel_mag_g: float
    accel_range_g: int = Field(ge=0)
    accel_id: str = Field(max_length=16)
    accel_chip: str = Field(default="", max_length=32)
    uptime_s: int = Field(ge=0)
    cpu_mhz: int = Field(ge=0)
    rssi: int
    ip: str = Field(max_length=45)
    lcd_frames: int = Field(default=0, ge=0)
    lcd_fps: float = Field(default=0.0, ge=0)
    lcd_err: int = Field(default=0, ge=0)


class TelemetryBatch(BaseModel):
    device_id: str = Field(min_length=1, max_length=64)
    samples: list[TelemetrySample] = Field(min_length=1, max_length=20)


class CaptureTaskCreate(BaseModel):
    device_id: str = Field(min_length=1, max_length=64)
    task_type: Literal["capture_full_snapshot", "capture_photo"] = "capture_full_snapshot"


class TaskAck(BaseModel):
    device_id: str = Field(min_length=1, max_length=64)


class TaskResult(TaskAck):
    sample: TelemetrySample


class TaskFailure(TaskAck):
    error: str = Field(min_length=1, max_length=240)


class PeriodicIngestionControl(BaseModel):
    paused: bool


class DeviceEventCreate(BaseModel):
    """A locally-confirmed button action uploaded by one ESP32 device."""
    device_id: str = Field(min_length=1, max_length=64)
    event_id: str = Field(min_length=36, max_length=36)
    event_type: Literal["help_request", "test_message"]
    occurred_at_ms: int = Field(ge=0)


class DeviceEventDeviceRef(BaseModel):
    device_id: str = Field(min_length=1, max_length=64)


class DeviceEventResponse(BaseModel):
    message: str = Field(default="", max_length=280)


SCHEMA = """
CREATE TABLE IF NOT EXISTS telemetry (
  id INTEGER PRIMARY KEY,
  device_id TEXT NOT NULL,
  boot_id TEXT NOT NULL,
  sequence INTEGER NOT NULL,
  captured_at_ms INTEGER NOT NULL,
  received_at_ms INTEGER NOT NULL,
  temp_c REAL NOT NULL, heap_int INTEGER NOT NULL, heap_psram INTEGER NOT NULL,
  fps REAL NOT NULL, jpeg INTEGER NOT NULL, accel_ok INTEGER NOT NULL,
  accel_x_g REAL NOT NULL, accel_y_g REAL NOT NULL, accel_z_g REAL NOT NULL,
  accel_mag_g REAL NOT NULL, accel_range_g INTEGER NOT NULL, accel_id TEXT NOT NULL, accel_chip TEXT NOT NULL,
  uptime_s INTEGER NOT NULL, cpu_mhz INTEGER NOT NULL, rssi INTEGER NOT NULL, ip TEXT NOT NULL,
  lcd_frames INTEGER NOT NULL DEFAULT 0, lcd_fps REAL NOT NULL DEFAULT 0, lcd_err INTEGER NOT NULL DEFAULT 0,
  source TEXT NOT NULL DEFAULT 'periodic', request_id TEXT,
  UNIQUE(device_id, boot_id, sequence)
);
CREATE INDEX IF NOT EXISTS telemetry_time_idx ON telemetry(device_id, captured_at_ms DESC);
CREATE TABLE IF NOT EXISTS devices (
  device_id TEXT PRIMARY KEY, last_seen_at_ms INTEGER NOT NULL, last_ip TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS service_settings (
  name TEXT PRIMARY KEY, value TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS capture_tasks (
  request_id TEXT PRIMARY KEY, device_id TEXT NOT NULL, task_type TEXT NOT NULL,
  sensor_sources TEXT NOT NULL, status TEXT NOT NULL,
  created_at_ms INTEGER NOT NULL, expires_at_ms INTEGER NOT NULL,
  received_at_ms INTEGER, completed_at_ms INTEGER, failed_at_ms INTEGER,
  error TEXT, telemetry_id INTEGER, photo_id INTEGER,
  FOREIGN KEY(telemetry_id) REFERENCES telemetry(id),
  FOREIGN KEY(photo_id) REFERENCES photos(id)
);
CREATE INDEX IF NOT EXISTS capture_tasks_status_idx ON capture_tasks(device_id, status, created_at_ms);
CREATE TABLE IF NOT EXISTS photos (
  id INTEGER PRIMARY KEY,
  request_id TEXT NOT NULL UNIQUE,
  device_id TEXT NOT NULL,
  captured_at_ms INTEGER NOT NULL,
  received_at_ms INTEGER NOT NULL,
  content_type TEXT NOT NULL,
  byte_size INTEGER NOT NULL,
  sha256 TEXT NOT NULL,
  storage_name TEXT NOT NULL UNIQUE
);
CREATE INDEX IF NOT EXISTS photos_device_time_idx ON photos(device_id, received_at_ms DESC);
CREATE TABLE IF NOT EXISTS device_events (
  id INTEGER PRIMARY KEY,
  event_id TEXT NOT NULL,
  device_id TEXT NOT NULL,
  event_type TEXT NOT NULL,
  occurred_at_ms INTEGER NOT NULL,
  received_at_ms INTEGER NOT NULL,
  status TEXT NOT NULL,
  viewer_message TEXT,
  viewer_responded_at_ms INTEGER,
  cancelled_at_ms INTEGER,
  cancelled_by TEXT,
  device_acknowledged_at_ms INTEGER,
  UNIQUE(device_id, event_id)
);
CREATE INDEX IF NOT EXISTS device_events_device_time_idx ON device_events(device_id, received_at_ms DESC);
"""

# Columns added after the first release.  CREATE TABLE IF NOT EXISTS leaves an
# existing table untouched, so a database written by an earlier build would
# reject inserts naming these columns.  ALTER TABLE ... ADD COLUMN cannot add a
# NOT NULL column without a default, hence the explicit defaults.
ADDED_COLUMNS = {
    "accel_chip": "TEXT NOT NULL DEFAULT ''",
    "lcd_frames": "INTEGER NOT NULL DEFAULT 0",
    "lcd_fps": "REAL NOT NULL DEFAULT 0",
    "lcd_err": "INTEGER NOT NULL DEFAULT 0",
    "source": "TEXT NOT NULL DEFAULT 'periodic'",
    "request_id": "TEXT",
}

TASK_ADDED_COLUMNS = {
    "photo_id": "INTEGER",
}

PHOTO_WIDTH = 240
PHOTO_HEIGHT = 240
PHOTO_HEADER_SIZE = 54
PHOTO_BYTE_SIZE = PHOTO_HEADER_SIZE + PHOTO_WIDTH * PHOTO_HEIGHT * 3
PHOTO_RETENTION_MS = 7 * 86400 * 1000


def add_missing_columns(conn: sqlite3.Connection) -> None:
    present = {row[1] for row in conn.execute("PRAGMA table_info(telemetry)")}
    for name, declaration in ADDED_COLUMNS.items():
        if name not in present:
            conn.execute(f"ALTER TABLE telemetry ADD COLUMN {name} {declaration}")
    task_columns = {row[1] for row in conn.execute("PRAGMA table_info(capture_tasks)")}
    for name, declaration in TASK_ADDED_COLUMNS.items():
        if name not in task_columns:
            conn.execute(f"ALTER TABLE capture_tasks ADD COLUMN {name} {declaration}")


class Store:
    def __init__(self, database: Path) -> None:
        self.database = database
        self.database.parent.mkdir(parents=True, exist_ok=True)
        self.photo_dir = self.database.parent / "photos"
        self.photo_dir.mkdir(parents=True, exist_ok=True)
        self._cleanup_lock = threading.Lock()
        self._live_lock = threading.Lock()
        self._live_samples: dict[str, dict] = {}
        self._last_cleanup = 0.0
        with self.connect() as conn:
            conn.executescript(SCHEMA)
            add_missing_columns(conn)
            conn.execute("CREATE INDEX IF NOT EXISTS telemetry_request_idx ON telemetry(request_id)")
            conn.execute("PRAGMA journal_mode=WAL")
            conn.execute("PRAGMA synchronous=NORMAL")
        self.cleanup(force=True)

    @contextmanager
    def connect(self):
        conn = sqlite3.connect(self.database)
        conn.row_factory = sqlite3.Row
        try:
            yield conn
            conn.commit()
        finally:
            conn.close()

    def cleanup(self, force: bool = False) -> None:
        now = time.time()
        if not force and now - self._last_cleanup < 86400:
            return
        with self._cleanup_lock:
            if not force and now - self._last_cleanup < 86400:
                return
            cutoff = int((now - 30 * 86400) * 1000)
            with self.connect() as conn:
                # Unknown device timestamps (0 before SNTP) are retained until received data ages out.
                conn.execute(
                    "DELETE FROM telemetry WHERE "
                    "(captured_at_ms > 0 AND captured_at_ms < ?) OR "
                    "(captured_at_ms = 0 AND received_at_ms < ?)",
                    (cutoff, cutoff),
                )
                conn.execute(
                    "DELETE FROM device_events WHERE "
                    "(occurred_at_ms > 0 AND occurred_at_ms < ?) OR "
                    "(occurred_at_ms = 0 AND received_at_ms < ?)",
                    (cutoff, cutoff),
                )
                expired = conn.execute(
                    "SELECT id, storage_name FROM photos WHERE received_at_ms < ?",
                    (int(time.time() * 1000) - PHOTO_RETENTION_MS,),
                ).fetchall()
                for photo in expired:
                    path = self.photo_dir / photo["storage_name"]
                    try:
                        path.unlink()
                    except FileNotFoundError:
                        pass
                    conn.execute("UPDATE capture_tasks SET photo_id=NULL WHERE photo_id=?", (photo["id"],))
                    conn.execute("DELETE FROM photos WHERE id=?", (photo["id"],))
            self._last_cleanup = now

    def _insert_rows(self, conn: sqlite3.Connection, batch: TelemetryBatch, source: str = "periodic",
                     request_id: str | None = None) -> list[int]:
        received_at_ms = int(time.time() * 1000)
        rows = [
            (
                batch.device_id, s.boot_id, s.sequence, s.captured_at_ms, received_at_ms,
                s.temp_c, s.heap_int, s.heap_psram, s.fps, s.jpeg, int(s.accel_ok),
                s.accel_x_g, s.accel_y_g, s.accel_z_g, s.accel_mag_g, s.accel_range_g,
                s.accel_id, s.accel_chip or {"0x90": "QMA6100P", "0xE7": "QMA7981"}.get(s.accel_id, "Unknown"),
                s.uptime_s, s.cpu_mhz, s.rssi, s.ip, s.lcd_frames, s.lcd_fps, s.lcd_err, source, request_id,
            )
            for s in batch.samples
        ]
        sql = """
            INSERT OR IGNORE INTO telemetry (
              device_id, boot_id, sequence, captured_at_ms, received_at_ms,
              temp_c, heap_int, heap_psram, fps, jpeg, accel_ok,
              accel_x_g, accel_y_g, accel_z_g, accel_mag_g, accel_range_g,
              accel_id, accel_chip, uptime_s, cpu_mhz, rssi, ip, lcd_frames, lcd_fps, lcd_err, source, request_id
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """
        ids = []
        for row in rows:
            conn.execute(sql, row)
            found = conn.execute("SELECT id FROM telemetry WHERE device_id=? AND boot_id=? AND sequence=?",
                                 row[:3]).fetchone()
            if found:
                ids.append(found[0])
        return ids

    @staticmethod
    def _upsert_device(conn: sqlite3.Connection, device_id: str, ip: str) -> None:
        conn.execute("INSERT INTO devices(device_id,last_seen_at_ms,last_ip) VALUES(?,?,?) "
                     "ON CONFLICT(device_id) DO UPDATE SET last_seen_at_ms=excluded.last_seen_at_ms,last_ip=excluded.last_ip",
                     (device_id, int(time.time() * 1000), ip))

    def periodic_paused(self, conn: sqlite3.Connection) -> bool:
        row = conn.execute("SELECT value FROM service_settings WHERE name='periodic_ingestion_paused'").fetchone()
        return row is not None and row[0] == "1"

    def set_periodic_paused(self, paused: bool) -> None:
        with self.connect() as conn:
            conn.execute("INSERT INTO service_settings(name,value) VALUES('periodic_ingestion_paused',?) "
                         "ON CONFLICT(name) DO UPDATE SET value=excluded.value", ("1" if paused else "0",))

    def insert_batch(self, batch: TelemetryBatch) -> int:
        with self.connect() as conn:
            self._upsert_device(conn, batch.device_id, batch.samples[-1].ip)
            if self.periodic_paused(conn):
                return 0
            before = conn.total_changes
            self._insert_rows(conn, batch)
            return conn.total_changes - before

    def publish_live(self, batch: TelemetryBatch) -> dict:
        """Keep one current sample per device without growing SQLite history."""
        received_at_ms = int(time.time() * 1000)
        sample = batch.samples[-1].model_dump()
        sample.update(device_id=batch.device_id, received_at_ms=received_at_ms, source="live")
        with self._live_lock:
            self._live_samples[batch.device_id] = sample
        with self.connect() as conn:
            self._upsert_device(conn, batch.device_id, sample["ip"])
        return sample

    def latest_live(self, device_id: str | None = None) -> dict | None:
        with self._live_lock:
            if device_id:
                sample = self._live_samples.get(device_id)
            elif self._live_samples:
                sample = max(self._live_samples.values(), key=lambda value: value["received_at_ms"])
            else:
                sample = None
            return dict(sample) if sample else None

    @staticmethod
    def _expire_tasks(conn: sqlite3.Connection) -> None:
        now = int(time.time() * 1000)
        conn.execute("UPDATE capture_tasks SET status='timeout', failed_at_ms=?, error='任务在 30 秒内未完成' "
                     "WHERE status IN ('submitted','received') AND expires_at_ms <= ?", (now, now))

    def create_task(self, device_id: str, task_type: str) -> dict:
        now = int(time.time() * 1000)
        task = {"request_id": str(uuid.uuid4()), "device_id": device_id,
                "task_type": task_type,
                "sensor_sources": ("camera" if task_type == "capture_photo"
                                   else "temperature,heap,imu,wifi,camera,lcd"),
                "status": "submitted", "created_at_ms": now, "expires_at_ms": now + 30000}
        with self.connect() as conn:
            conn.execute("INSERT INTO capture_tasks(request_id,device_id,task_type,sensor_sources,status,created_at_ms,expires_at_ms) "
                         "VALUES(:request_id,:device_id,:task_type,:sensor_sources,:status,:created_at_ms,:expires_at_ms)", task)
        return task

    def next_task(self, device_id: str) -> dict | None:
        with self.connect() as conn:
            self._expire_tasks(conn)
            row = conn.execute("SELECT * FROM capture_tasks WHERE device_id=? AND status IN ('submitted','received') "
                               "ORDER BY created_at_ms LIMIT 1", (device_id,)).fetchone()
            return dict(row) if row else None

    def ack_task(self, request_id: str, device_id: str) -> dict:
        now = int(time.time() * 1000)
        with self.connect() as conn:
            self._expire_tasks(conn)
            row = conn.execute("SELECT * FROM capture_tasks WHERE request_id=? AND device_id=?", (request_id, device_id)).fetchone()
            if not row: raise KeyError("任务不存在或不属于该设备")
            if row["status"] == "submitted":
                conn.execute("UPDATE capture_tasks SET status='received', received_at_ms=? WHERE request_id=?", (now, request_id))
            elif row["status"] not in ("received", "completed"):
                raise ValueError(row["status"])
            updated = conn.execute("SELECT * FROM capture_tasks WHERE request_id=?", (request_id,)).fetchone()
            return dict(updated)

    def complete_task(self, request_id: str, result: TaskResult) -> dict:
        with self.connect() as conn:
            self._expire_tasks(conn)
            row = conn.execute("SELECT * FROM capture_tasks WHERE request_id=? AND device_id=?", (request_id, result.device_id)).fetchone()
            if not row: raise KeyError("任务不存在或不属于该设备")
            if row["status"] == "completed": return dict(row)
            if row["task_type"] != "capture_full_snapshot": raise ValueError("不是传感器快照任务")
            if row["status"] not in ("submitted", "received"): raise ValueError(row["status"])
            batch = TelemetryBatch(device_id=result.device_id, samples=[result.sample])
            self._upsert_device(conn, result.device_id, result.sample.ip)
            telemetry_id = self._insert_rows(conn, batch, "on_demand", request_id)[0]
            now = int(time.time() * 1000)
            conn.execute("UPDATE capture_tasks SET status='completed', completed_at_ms=?, telemetry_id=? WHERE request_id=?",
                         (now, telemetry_id, request_id))
            return dict(conn.execute("SELECT * FROM capture_tasks WHERE request_id=?", (request_id,)).fetchone())

    @staticmethod
    def validate_bmp(payload: bytes) -> None:
        if len(payload) != PHOTO_BYTE_SIZE or payload[:2] != b"BM":
            raise ValueError("照片必须是完整的 240x240 BMP")
        little = lambda offset, length, signed=False: int.from_bytes(payload[offset:offset + length], "little", signed=signed)
        if (little(2, 4) != PHOTO_BYTE_SIZE or little(10, 4) != PHOTO_HEADER_SIZE or
                little(14, 4) != 40 or little(18, 4, signed=True) != PHOTO_WIDTH or
                abs(little(22, 4, signed=True)) != PHOTO_HEIGHT or little(26, 2) != 1 or
                little(28, 2) != 24 or little(30, 4) != 0 or little(34, 4) != PHOTO_WIDTH * PHOTO_HEIGHT * 3):
            raise ValueError("BMP 头或尺寸不符合 ESP32-S3-EYE 照片格式")

    def complete_photo_task(self, request_id: str, device_id: str, captured_at_ms: int, payload: bytes) -> dict:
        self.validate_bmp(payload)
        storage_name = f"{request_id}.bmp"
        final_path = self.photo_dir / storage_name
        temp_path = self.photo_dir / f".{request_id}.{uuid.uuid4().hex}.upload"
        temp_path.write_bytes(payload)
        moved = False
        try:
            with self.connect() as conn:
                self._expire_tasks(conn)
                row = conn.execute("SELECT * FROM capture_tasks WHERE request_id=? AND device_id=?", (request_id, device_id)).fetchone()
                if not row:
                    raise KeyError("任务不存在或不属于该设备")
                if row["status"] == "completed":
                    return dict(row)
                if row["task_type"] != "capture_photo":
                    raise ValueError("不是拍照任务")
                if row["status"] not in ("submitted", "received"):
                    raise ValueError(row["status"])
                os.replace(temp_path, final_path)
                moved = True
                now = int(time.time() * 1000)
                photo_id = conn.execute(
                    "INSERT INTO photos(request_id,device_id,captured_at_ms,received_at_ms,content_type,byte_size,sha256,storage_name) "
                    "VALUES(?,?,?,?,?,?,?,?)",
                    (request_id, device_id, captured_at_ms, now, "image/bmp", len(payload),
                     hashlib.sha256(payload).hexdigest(), storage_name),
                ).lastrowid
                conn.execute("UPDATE capture_tasks SET status='completed', completed_at_ms=?, photo_id=? WHERE request_id=?",
                             (now, photo_id, request_id))
                return dict(conn.execute("SELECT * FROM capture_tasks WHERE request_id=?", (request_id,)).fetchone())
        except Exception:
            if moved:
                try:
                    final_path.unlink()
                except FileNotFoundError:
                    pass
            raise
        finally:
            try:
                temp_path.unlink()
            except FileNotFoundError:
                pass

    def fail_task(self, request_id: str, failure: TaskFailure) -> dict:
        with self.connect() as conn:
            self._expire_tasks(conn)
            row = conn.execute("SELECT * FROM capture_tasks WHERE request_id=? AND device_id=?", (request_id, failure.device_id)).fetchone()
            if not row: raise KeyError("任务不存在或不属于该设备")
            if row["status"] in ("completed", "timeout"): raise ValueError(row["status"])
            now = int(time.time() * 1000)
            conn.execute("UPDATE capture_tasks SET status='failed', failed_at_ms=?, error=? WHERE request_id=?",
                         (now, failure.error, request_id))
            return dict(conn.execute("SELECT * FROM capture_tasks WHERE request_id=?", (request_id,)).fetchone())

    def list_tasks(self, device_id: str | None = None) -> list[dict]:
        with self.connect() as conn:
            self._expire_tasks(conn)
            if device_id:
                rows = conn.execute("SELECT * FROM capture_tasks WHERE device_id=? ORDER BY created_at_ms DESC LIMIT 100", (device_id,))
            else:
                rows = conn.execute("SELECT * FROM capture_tasks ORDER BY created_at_ms DESC LIMIT 100")
            return [dict(row) for row in rows]

    def list_devices(self) -> list[dict]:
        with self.connect() as conn:
            return [dict(row) for row in conn.execute("SELECT * FROM devices ORDER BY last_seen_at_ms DESC")]

    def list_photos(self, device_id: str | None = None) -> list[dict]:
        with self.connect() as conn:
            if device_id:
                rows = conn.execute("SELECT * FROM photos WHERE device_id=? ORDER BY received_at_ms DESC LIMIT 100", (device_id,))
            else:
                rows = conn.execute("SELECT * FROM photos ORDER BY received_at_ms DESC LIMIT 100")
            return [dict(row) for row in rows]

    def photo_path(self, photo_id: int) -> Path | None:
        with self.connect() as conn:
            row = conn.execute("SELECT storage_name FROM photos WHERE id=?", (photo_id,)).fetchone()
            if not row:
                return None
            path = self.photo_dir / row["storage_name"]
            return path if path.is_file() else None

    def create_device_event(self, event: DeviceEventCreate) -> dict:
        """Persist a button event before telling the board that it reached us.

        The device owns event_id generation. Returning an existing row for the
        same (device, event_id) makes retries after a lost HTTP response safe.
        """
        now = int(time.time() * 1000)
        with self.connect() as conn:
            conn.execute(
                "INSERT OR IGNORE INTO device_events("
                "event_id,device_id,event_type,occurred_at_ms,received_at_ms,status) "
                "VALUES(?,?,?,?,?, 'received')",
                (event.event_id, event.device_id, event.event_type, event.occurred_at_ms, now),
            )
            row = conn.execute(
                "SELECT * FROM device_events WHERE device_id=? AND event_id=?",
                (event.device_id, event.event_id),
            ).fetchone()
            return dict(row)

    @staticmethod
    def _event_for_device(conn: sqlite3.Connection, event_id: str, device_id: str) -> sqlite3.Row:
        row = conn.execute(
            "SELECT * FROM device_events WHERE event_id=? AND device_id=?", (event_id, device_id)
        ).fetchone()
        if not row:
            raise KeyError("事件不存在或不属于该设备")
        return row

    def get_device_event(self, event_id: str, device_id: str) -> dict:
        with self.connect() as conn:
            return dict(self._event_for_device(conn, event_id, device_id))

    def cancel_device_event(self, event_id: str, device_id: str, cancelled_by: str) -> dict:
        with self.connect() as conn:
            row = self._event_for_device(conn, event_id, device_id)
            if row["status"] == "cancelled":
                return dict(row)
            if row["status"] == "responded":
                raise ValueError("查看者已经回应，不能取消")
            now = int(time.time() * 1000)
            conn.execute(
                "UPDATE device_events SET status='cancelled', cancelled_at_ms=?, cancelled_by=? "
                "WHERE id=?", (now, cancelled_by, row["id"])
            )
            return dict(conn.execute("SELECT * FROM device_events WHERE id=?", (row["id"],)).fetchone())

    def respond_device_event(self, event_id: str, message: str) -> dict:
        with self.connect() as conn:
            row = conn.execute("SELECT * FROM device_events WHERE event_id=?", (event_id,)).fetchone()
            if not row:
                raise KeyError("事件不存在")
            if row["status"] == "responded":
                return dict(row)
            if row["status"] == "cancelled":
                raise ValueError("事件已取消")
            now = int(time.time() * 1000)
            conn.execute(
                "UPDATE device_events SET status='responded', viewer_message=?, viewer_responded_at_ms=? WHERE id=?",
                (message, now, row["id"]),
            )
            return dict(conn.execute("SELECT * FROM device_events WHERE id=?", (row["id"],)).fetchone())

    def acknowledge_device_event_delivery(self, event_id: str, device_id: str) -> dict:
        with self.connect() as conn:
            row = self._event_for_device(conn, event_id, device_id)
            if row["status"] not in ("responded", "cancelled"):
                raise ValueError("查看者尚未回应或取消")
            if row["device_acknowledged_at_ms"] is None:
                conn.execute(
                    "UPDATE device_events SET device_acknowledged_at_ms=? WHERE id=?",
                    (int(time.time() * 1000), row["id"]),
                )
            return dict(conn.execute("SELECT * FROM device_events WHERE id=?", (row["id"],)).fetchone())

    def list_device_events(self, device_id: str | None = None) -> list[dict]:
        with self.connect() as conn:
            if device_id:
                rows = conn.execute(
                    "SELECT * FROM device_events WHERE device_id=? ORDER BY received_at_ms DESC LIMIT 100",
                    (device_id,),
                )
            else:
                rows = conn.execute("SELECT * FROM device_events ORDER BY received_at_ms DESC LIMIT 100")
            return [dict(row) for row in rows]

    def query(self, start_ms: int | None, end_ms: int | None, device_id: str | None, limit: int):
        where, values = [], []
        if start_ms is not None:
            where.append("COALESCE(NULLIF(captured_at_ms, 0), received_at_ms) >= ?")
            values.append(start_ms)
        if end_ms is not None:
            where.append("COALESCE(NULLIF(captured_at_ms, 0), received_at_ms) <= ?")
            values.append(end_ms)
        if device_id:
            where.append("device_id = ?")
            values.append(device_id)
        clause = " WHERE " + " AND ".join(where) if where else ""
        sql = "SELECT * FROM telemetry" + clause + " ORDER BY COALESCE(NULLIF(captured_at_ms, 0), received_at_ms) ASC LIMIT ?"
        values.append(limit)
        with self.connect() as conn:
            return [dict(row) for row in conn.execute(sql, values)]


def create_app(database: Path | None = None, api_key: str | None = None,
               dashboard_host: str | None = None, ingest_host: str | None = None) -> FastAPI:
    store = Store(database or ROOT / "data" / "telemetry.sqlite3")
    key = api_key or os.environ.get("TELEMETRY_API_KEY", "CHANGE_ME")
    dashboard_host = (dashboard_host or os.environ.get("REMOTE_DASHBOARD_HOST", "")).lower().strip()
    ingest_host = (ingest_host or os.environ.get("REMOTE_INGEST_HOST", "")).lower().strip()
    if bool(dashboard_host) != bool(ingest_host):
        raise RuntimeError("REMOTE_DASHBOARD_HOST and REMOTE_INGEST_HOST must be set together")
    app = FastAPI(title="ESP32 Telemetry Collector", version="1.0")
    app.state.store = store
    app.mount("/static", StaticFiles(directory=ROOT / "static"), name="static")

    def require_api_key(provided: str | None = Header(default=None, alias="X-Api-Key")) -> None:
        if not provided or not __import__("hmac").compare_digest(provided, key):
            raise HTTPException(status_code=401, detail="invalid API key")

    def is_device_endpoint(request: Request) -> bool:
        path, method = request.url.path, request.method
        if method == "POST" and path in ("/api/v1/telemetry", "/api/v1/live-telemetry"):
            return True
        if method == "GET" and path.startswith("/api/v1/devices/") and path.endswith("/tasks/next"):
            return True
        if method == "POST" and path == "/api/v1/device-events":
            return True
        if method == "GET" and path.startswith("/api/v1/device-events/") and path.endswith("/device"):
            return True
        if method == "POST" and path.startswith("/api/v1/device-events/") and \
                path.rsplit("/", 1)[-1] in {"cancel", "delivery-ack"}:
            return True
        return (method == "POST" and path.startswith("/api/v1/capture-tasks/") and
                path.rsplit("/", 1)[-1] in {"ack", "result", "photo", "fail"})

    @app.middleware("http")
    async def isolate_remote_hosts(request: Request, call_next):
        """The dashboard is protected by Cloudflare Access; the ingest host is
        deliberately smaller and relies on the ESP32 API key for every route."""
        if not dashboard_host:
            return await call_next(request)  # local development / existing LAN deployment
        host = request.headers.get("host", "").split(":", 1)[0].lower()
        if host == dashboard_host:
            return await call_next(request)
        if host == ingest_host:
            if is_device_endpoint(request):
                return await call_next(request)
            return JSONResponse(status_code=404, content={"detail": "not available on ingest host"})
        return JSONResponse(status_code=421, content={"detail": "unknown host"})

    @app.post("/api/v1/telemetry", status_code=200)
    def post_telemetry(batch: TelemetryBatch, _: None = Depends(require_api_key)):
        store.cleanup()
        inserted = store.insert_batch(batch)
        return {"accepted": len(batch.samples), "inserted": inserted}

    @app.post("/api/v1/live-telemetry", status_code=200)
    def post_live_telemetry(batch: TelemetryBatch, _: None = Depends(require_api_key)):
        sample = store.publish_live(batch)
        return {"accepted": len(batch.samples), "received_at_ms": sample["received_at_ms"]}

    @app.get("/api/v1/live-telemetry")
    def get_live_telemetry(device_id: str | None = None):
        return {"sample": store.latest_live(device_id)}

    @app.get("/api/v1/devices")
    def get_devices():
        return {"devices": store.list_devices()}

    @app.post("/api/v1/device-events", status_code=201)
    def create_device_event(event: DeviceEventCreate, _: None = Depends(require_api_key)):
        store.cleanup()
        return store.create_device_event(event)

    @app.get("/api/v1/device-events")
    def get_device_events(device_id: str | None = None):
        store.cleanup()
        return {"events": store.list_device_events(device_id)}

    @app.get("/api/v1/device-events/{event_id}/device")
    def get_device_event(event_id: str, device_id: str, _: None = Depends(require_api_key)):
        try:
            return store.get_device_event(event_id, device_id)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc

    @app.post("/api/v1/device-events/{event_id}/cancel")
    def device_cancel_event(event_id: str, event: DeviceEventDeviceRef, _: None = Depends(require_api_key)):
        try:
            return store.cancel_device_event(event_id, event.device_id, "device")
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/v1/device-events/{event_id}/delivery-ack")
    def acknowledge_event_delivery(event_id: str, event: DeviceEventDeviceRef, _: None = Depends(require_api_key)):
        try:
            return store.acknowledge_device_event_delivery(event_id, event.device_id)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.put("/api/v1/device-events/{event_id}/respond")
    def respond_event(event_id: str, response: DeviceEventResponse):
        try:
            return store.respond_device_event(event_id, response.message.strip())
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.put("/api/v1/device-events/{event_id}/cancel")
    def viewer_cancel_event(event_id: str):
        try:
            with store.connect() as conn:
                row = conn.execute("SELECT device_id FROM device_events WHERE event_id=?", (event_id,)).fetchone()
            if not row:
                raise KeyError("事件不存在")
            return store.cancel_device_event(event_id, row["device_id"], "viewer")
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/v1/capture-tasks", status_code=201)
    def create_capture_task(request: CaptureTaskCreate):
        return store.create_task(request.device_id, request.task_type)

    @app.get("/api/v1/capture-tasks")
    def get_capture_tasks(device_id: str | None = None):
        return {"tasks": store.list_tasks(device_id)}

    @app.get("/api/v1/capture-tasks/{request_id}")
    def get_capture_task(request_id: str):
        for task in store.list_tasks():
            if task["request_id"] == request_id:
                return task
        raise HTTPException(status_code=404, detail="task not found")

    @app.get("/api/v1/devices/{device_id}/tasks/next")
    def get_next_task(device_id: str, _: None = Depends(require_api_key)):
        return {"task": store.next_task(device_id)}

    @app.post("/api/v1/capture-tasks/{request_id}/ack")
    def acknowledge_capture_task(request_id: str, ack: TaskAck, _: None = Depends(require_api_key)):
        try:
            return store.ack_task(request_id, ack.device_id)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=f"task is {exc}") from exc

    @app.post("/api/v1/capture-tasks/{request_id}/result")
    def complete_capture_task(request_id: str, result: TaskResult, _: None = Depends(require_api_key)):
        try:
            return store.complete_task(request_id, result)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=f"task is {exc}") from exc

    @app.post("/api/v1/capture-tasks/{request_id}/photo")
    async def complete_capture_photo(
        request_id: str,
        request: Request,
        device_id: str = Header(alias="X-Device-Id"),
        captured_at_ms: int = Header(alias="X-Captured-At-Ms"),
        _: None = Depends(require_api_key),
    ):
        if request.headers.get("content-type", "").split(";", 1)[0].lower() != "image/bmp":
            raise HTTPException(status_code=415, detail="photo content type must be image/bmp")
        payload = await request.body()
        try:
            return store.complete_photo_task(request_id, device_id, captured_at_ms, payload)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=str(exc)) from exc

    @app.post("/api/v1/capture-tasks/{request_id}/fail")
    def fail_capture_task(request_id: str, failure: TaskFailure, _: None = Depends(require_api_key)):
        try:
            return store.fail_task(request_id, failure)
        except KeyError as exc:
            raise HTTPException(status_code=404, detail=str(exc)) from exc
        except ValueError as exc:
            raise HTTPException(status_code=409, detail=f"task is {exc}") from exc

    @app.get("/api/v1/control/periodic-ingestion")
    def get_periodic_ingestion_state():
        with store.connect() as conn:
            return {"paused": store.periodic_paused(conn)}

    @app.put("/api/v1/control/periodic-ingestion")
    def set_periodic_ingestion_state(control: PeriodicIngestionControl):
        store.set_periodic_paused(control.paused)
        return {"paused": control.paused}

    @app.get("/api/v1/photos")
    def get_photos(device_id: str | None = None):
        store.cleanup()
        return {"photos": store.list_photos(device_id)}

    @app.get("/api/v1/photos/{photo_id}/image")
    def get_photo_image(photo_id: int):
        store.cleanup()
        path = store.photo_path(photo_id)
        if path is None:
            raise HTTPException(status_code=404, detail="photo not found or expired")
        return FileResponse(path, media_type="image/bmp")

    @app.get("/api/v1/telemetry")
    def get_telemetry(
        start_ms: int | None = None,
        end_ms: int | None = None,
        device_id: str | None = None,
        limit: int = Query(default=20000, ge=1, le=100000),
    ):
        return {"samples": store.query(start_ms, end_ms, device_id, limit)}

    @app.get("/api/v1/export.csv")
    def export_csv(start_ms: int | None = None, end_ms: int | None = None, device_id: str | None = None):
        rows = store.query(start_ms, end_ms, device_id, 100000)
        output = io.StringIO(newline="")
        fields = list(rows[0]) if rows else ["id", "device_id", "boot_id", "sequence", "captured_at_ms", "received_at_ms"]
        writer = csv.DictWriter(output, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
        return StreamingResponse(iter([output.getvalue()]), media_type="text/csv; charset=utf-8",
                                 headers={"Content-Disposition": "attachment; filename=telemetry.csv"})

    @app.get("/")
    def dashboard():
        return FileResponse(ROOT / "static" / "index.html")

    return app


app = create_app()
