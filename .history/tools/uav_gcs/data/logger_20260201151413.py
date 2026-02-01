"""
UAV Ground Control Station - Data Logger

SQLite-based logging for offline analysis.
Runs in separate thread to avoid blocking.
"""

import sqlite3
import threading
import queue
import time
from pathlib import Path
from typing import Optional
from datetime import datetime

from ..protocol.packet import TelemetryData


class DataLogger:
    """
    Async SQLite logger for telemetry data.
    
    Uses queue + worker thread to avoid blocking.
    """
    
    def __init__(self, log_dir: str = "./logs"):
        self._log_dir = Path(log_dir)
        self._log_dir.mkdir(parents=True, exist_ok=True)
        
        self._db_path: Optional[Path] = None
        self._conn: Optional[sqlite3.Connection] = None
        self._session_id: Optional[int] = None
        
        # Async logging
        self._queue: queue.Queue[Optional[TelemetryData]] = queue.Queue(maxsize=10000)
        self._thread: Optional[threading.Thread] = None
        self._running = False
        
        # Stats
        self.records_logged = 0
        self.queue_drops = 0
    
    def start_session(self, description: str = "") -> str:
        """
        Start new logging session.
        
        Returns database file path.
        """
        if self._running:
            self.stop_session()
        
        # Create new database file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._db_path = self._log_dir / f"telemetry_{timestamp}.db"
        
        self._conn = sqlite3.connect(str(self._db_path), check_same_thread=False)
        self._create_schema()
        self._session_id = self._create_session(description)
        
        # Start worker thread
        self._running = True
        self._thread = threading.Thread(target=self._worker_loop, daemon=True)
        self._thread.start()
        
        return str(self._db_path)
    
    def stop_session(self):
        """Stop current logging session"""
        if not self._running:
            return
        
        self._running = False
        
        # Signal worker to stop
        self._queue.put(None)
        
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        
        # Close session
        if self._conn is not None and self._session_id is not None:
            try:
                self._conn.execute(
                    "UPDATE sessions SET end_time = ? WHERE id = ?",
                    (datetime.now().isoformat(), self._session_id)
                )
                self._conn.commit()
                self._conn.close()
            except:
                pass
        
        self._conn = None
        self._session_id = None
    
    def log(self, data: TelemetryData):
        """
        Queue data for logging (non-blocking).
        """
        if not self._running:
            return
        
        try:
            self._queue.put_nowait(data)
        except queue.Full:
            self.queue_drops += 1
    
    def _create_schema(self):
        """Create database schema"""
        cursor = self._conn.cursor()
        
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS sessions (
                id INTEGER PRIMARY KEY,
                start_time TEXT,
                end_time TEXT,
                description TEXT
            )
        """)
        
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS telemetry (
                id INTEGER PRIMARY KEY,
                session_id INTEGER,
                timestamp_us INTEGER,
                pc_time_ms INTEGER,
                sequence INTEGER,
                
                gyro_x REAL, gyro_y REAL, gyro_z REAL,
                accel_x REAL, accel_y REAL, accel_z REAL,
                imu_temp REAL,
                
                mag_x REAL, mag_y REAL, mag_z REAL,
                
                pressure REAL, baro_alt REAL,
                
                latitude REAL, longitude REAL, altitude_msl REAL,
                ground_speed REAL, heading REAL,
                fix_type INTEGER, satellites INTEGER,
                
                qw REAL, qx REAL, qy REAL, qz REAL,
                roll REAL, pitch REAL, yaw REAL,
                
                health_level INTEGER,
                healthy_imus INTEGER,
                sensor_flags INTEGER,
                cpu_load REAL,
                
                FOREIGN KEY (session_id) REFERENCES sessions(id)
            )
        """)
        
        cursor.execute("""
            CREATE INDEX IF NOT EXISTS idx_telemetry_time 
            ON telemetry(session_id, timestamp_us)
        """)
        
        self._conn.commit()
    
    def _create_session(self, description: str) -> int:
        """Create new session record"""
        cursor = self._conn.cursor()
        cursor.execute(
            "INSERT INTO sessions (start_time, description) VALUES (?, ?)",
            (datetime.now().isoformat(), description)
        )
        self._conn.commit()
        return cursor.lastrowid
    
    def _worker_loop(self):
        """Worker thread for async logging"""
        batch = []
        batch_size = 100
        last_flush = time.time()
        flush_interval = 1.0  # seconds
        
        while self._running or not self._queue.empty():
            try:
                data = self._queue.get(timeout=0.1)
                
                if data is None:
                    break
                
                batch.append(data)
                
                # Flush batch
                if len(batch) >= batch_size or (time.time() - last_flush) >= flush_interval:
                    self._flush_batch(batch)
                    batch.clear()
                    last_flush = time.time()
                
            except queue.Empty:
                # Flush any remaining
                if batch:
                    self._flush_batch(batch)
                    batch.clear()
                    last_flush = time.time()
        
        # Final flush
        if batch:
            self._flush_batch(batch)
    
    def _flush_batch(self, batch: list[TelemetryData]):
        """Write batch to database"""
        if not batch or self._conn is None:
            return
        
        cursor = self._conn.cursor()
        
        for data in batch:
            cursor.execute("""
                INSERT INTO telemetry (
                    session_id, timestamp_us, pc_time_ms, sequence,
                    gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z, imu_temp,
                    mag_x, mag_y, mag_z,
                    pressure, baro_alt,
                    latitude, longitude, altitude_msl, ground_speed, heading,
                    fix_type, satellites,
                    qw, qx, qy, qz, roll, pitch, yaw,
                    health_level, healthy_imus, sensor_flags, cpu_load
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """, (
                self._session_id, data.timestamp_us, data.pc_timestamp_ms, data.sequence,
                data.gyro_x, data.gyro_y, data.gyro_z,
                data.accel_x, data.accel_y, data.accel_z, data.imu_temp,
                data.mag_x, data.mag_y, data.mag_z,
                data.pressure, data.baro_alt,
                data.latitude, data.longitude, data.altitude_msl,
                data.ground_speed, data.heading,
                data.fix_type, data.satellites,
                data.qw, data.qx, data.qy, data.qz,
                data.roll, data.pitch, data.yaw,
                data.health_level, data.healthy_imus, data.sensor_flags, data.cpu_load
            ))
        
        self._conn.commit()
        self.records_logged += len(batch)
