"""LAN telemetry collector for the ESP32-S3-EYE dashboard."""
from __future__ import annotations

import csv
import io
import os
import sqlite3
import threading
import time
from contextlib import contextmanager
from pathlib import Path

from fastapi import Depends, FastAPI, Header, HTTPException, Query
from fastapi.responses import FileResponse, StreamingResponse
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
  UNIQUE(device_id, boot_id, sequence)
);
CREATE INDEX IF NOT EXISTS telemetry_time_idx ON telemetry(device_id, captured_at_ms DESC);
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
}


def add_missing_columns(conn: sqlite3.Connection) -> None:
    present = {row[1] for row in conn.execute("PRAGMA table_info(telemetry)")}
    for name, declaration in ADDED_COLUMNS.items():
        if name not in present:
            conn.execute(f"ALTER TABLE telemetry ADD COLUMN {name} {declaration}")


class Store:
    def __init__(self, database: Path) -> None:
        self.database = database
        self.database.parent.mkdir(parents=True, exist_ok=True)
        self._cleanup_lock = threading.Lock()
        self._last_cleanup = 0.0
        with self.connect() as conn:
            conn.executescript(SCHEMA)
            add_missing_columns(conn)
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
            self._last_cleanup = now

    def insert_batch(self, batch: TelemetryBatch) -> int:
        received_at_ms = int(time.time() * 1000)
        rows = [
            (
                batch.device_id, s.boot_id, s.sequence, s.captured_at_ms, received_at_ms,
                s.temp_c, s.heap_int, s.heap_psram, s.fps, s.jpeg, int(s.accel_ok),
                s.accel_x_g, s.accel_y_g, s.accel_z_g, s.accel_mag_g, s.accel_range_g,
                s.accel_id, s.accel_chip or {"0x90": "QMA6100P", "0xE7": "QMA7981"}.get(s.accel_id, "Unknown"),
                s.uptime_s, s.cpu_mhz, s.rssi, s.ip, s.lcd_frames, s.lcd_fps, s.lcd_err,
            )
            for s in batch.samples
        ]
        sql = """
            INSERT OR IGNORE INTO telemetry (
              device_id, boot_id, sequence, captured_at_ms, received_at_ms,
              temp_c, heap_int, heap_psram, fps, jpeg, accel_ok,
              accel_x_g, accel_y_g, accel_z_g, accel_mag_g, accel_range_g,
              accel_id, accel_chip, uptime_s, cpu_mhz, rssi, ip, lcd_frames, lcd_fps, lcd_err
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        """
        with self.connect() as conn:
            before = conn.total_changes
            conn.executemany(sql, rows)
            return conn.total_changes - before

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


def create_app(database: Path | None = None, api_key: str | None = None) -> FastAPI:
    store = Store(database or ROOT / "data" / "telemetry.sqlite3")
    key = api_key or os.environ.get("TELEMETRY_API_KEY", "CHANGE_ME")
    app = FastAPI(title="ESP32 Telemetry Collector", version="1.0")
    app.state.store = store
    app.mount("/static", StaticFiles(directory=ROOT / "static"), name="static")

    def require_api_key(provided: str | None = Header(default=None, alias="X-Api-Key")) -> None:
        if not provided or not __import__("hmac").compare_digest(provided, key):
            raise HTTPException(status_code=401, detail="invalid API key")

    @app.post("/api/v1/telemetry", status_code=200)
    def post_telemetry(batch: TelemetryBatch, _: None = Depends(require_api_key)):
        store.cleanup()
        inserted = store.insert_batch(batch)
        return {"accepted": len(batch.samples), "inserted": inserted}

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
