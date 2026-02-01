"""
UAV Ground Control Station - Data Manager

Quản lý data flow giữa receiver và GUI.
Thread-safe ring buffer cho history.
"""

import time
import threading
from collections import deque
from typing import Optional, List, Callable
from dataclasses import dataclass

from PySide6.QtCore import QObject, Signal, QTimer

from ..protocol.packet import TelemetryData


@dataclass
class DataStats:
    """Statistics for data management"""
    packets_received: int = 0
    packets_dropped: int = 0
    buffer_usage: float = 0.0
    update_rate_hz: float = 0.0
    latency_ms: float = 0.0


class RingBuffer:
    """
    Thread-safe ring buffer for telemetry history.
    
    Designed for multi-producer (receiver) single-consumer (GUI) pattern.
    """
    
    def __init__(self, max_size: int = 10000):
        self._buffer: deque[TelemetryData] = deque(maxlen=max_size)
        self._lock = threading.Lock()
        self._max_size = max_size
    
    def push(self, data: TelemetryData):
        """Add data to buffer (thread-safe)"""
        with self._lock:
            self._buffer.append(data)
    
    def get_last(self, n: int = 1) -> List[TelemetryData]:
        """Get last N items (thread-safe)"""
        with self._lock:
            if n >= len(self._buffer):
                return list(self._buffer)
            return list(self._buffer)[-n:]
    
    def get_all(self) -> List[TelemetryData]:
        """Get all items (thread-safe)"""
        with self._lock:
            return list(self._buffer)
    
    def get_range(self, start_time_us: int, end_time_us: int) -> List[TelemetryData]:
        """Get items within time range (thread-safe)"""
        with self._lock:
            return [d for d in self._buffer 
                    if start_time_us <= d.timestamp_us <= end_time_us]
    
    def clear(self):
        """Clear buffer (thread-safe)"""
        with self._lock:
            self._buffer.clear()
    
    def __len__(self) -> int:
        with self._lock:
            return len(self._buffer)
    
    @property
    def usage(self) -> float:
        """Buffer usage percentage"""
        return len(self) / self._max_size


class DataManager(QObject):
    """
    Central data manager.
    
    Coordinates data flow:
    - Receives packets from SerialReceiver
    - Stores in ring buffer
    - Emits signals for GUI updates
    - Manages logging
    """
    
    # Signals for GUI
    new_data = Signal(object)           # Latest TelemetryData
    stats_updated = Signal(object)      # DataStats
    
    def __init__(self, buffer_size: int = 10000, parent=None):
        super().__init__(parent)
        
        # Ring buffer for history
        self.buffer = RingBuffer(max_size=buffer_size)
        
        # Latest data (for quick access)
        self._latest: Optional[TelemetryData] = None
        self._latest_lock = threading.Lock()
        
        # Statistics
        self._packet_count = 0
        self._last_sequence = -1
        self._drops = 0
        self._rate_counter = 0
        self._rate_start_time = time.time()
        self._current_rate = 0.0
        
        # Logger callback
        self._log_callback: Optional[Callable[[TelemetryData], None]] = None
    
    def on_packet_received(self, data: TelemetryData):
        """
        Handle new packet from receiver.
        
        Called from receiver thread, must be thread-safe.
        """
        # Store in buffer
        self.buffer.push(data)
        
        # Update latest
        with self._latest_lock:
            self._latest = data
        
        # Update stats
        self._packet_count += 1
        self._rate_counter += 1
        
        # Check sequence drops
        if self._last_sequence >= 0:
            expected = (self._last_sequence + 1) & 0xFFFF
            if data.sequence != expected:
                gap = (data.sequence - expected) & 0xFFFF
                self._drops += gap
        self._last_sequence = data.sequence
        
        # Calculate rate
        now = time.time()
        elapsed = now - self._rate_start_time
        if elapsed >= 1.0:
            self._current_rate = self._rate_counter / elapsed
            self._rate_counter = 0
            self._rate_start_time = now
        
        # Emit signal (Qt will queue this for GUI thread)
        self.new_data.emit(data)
        
        # Log if enabled
        if self._log_callback is not None:
            try:
                self._log_callback(data)
            except Exception:
                pass
    
    def get_latest(self) -> Optional[TelemetryData]:
        """Get latest telemetry data (thread-safe)"""
        with self._latest_lock:
            return self._latest
    
    def get_history(self, duration_sec: float) -> List[TelemetryData]:
        """Get history for last N seconds"""
        all_data = self.buffer.get_all()
        if not all_data:
            return []
        
        latest_time = all_data[-1].timestamp_us
        cutoff = latest_time - int(duration_sec * 1_000_000)
        
        return [d for d in all_data if d.timestamp_us >= cutoff]
    
    def get_stats(self) -> DataStats:
        """Get current statistics"""
        return DataStats(
            packets_received=self._packet_count,
            packets_dropped=self._drops,
            buffer_usage=self.buffer.usage,
            update_rate_hz=self._current_rate,
            latency_ms=self._calculate_latency()
        )
    
    def _calculate_latency(self) -> float:
        """Calculate end-to-end latency"""
        latest = self.get_latest()
        if latest is None:
            return 0.0
        
        # PC time vs MCU timestamp (approximate)
        pc_time_us = latest.pc_timestamp_ms * 1000
        mcu_time_us = latest.timestamp_us
        
        # This is approximate since we don't have synchronized clocks
        # Just use as relative indicator
        return 0.0  # Placeholder
    
    def set_log_callback(self, callback: Optional[Callable[[TelemetryData], None]]):
        """Set logging callback"""
        self._log_callback = callback
    
    def clear(self):
        """Clear all data"""
        self.buffer.clear()
        with self._latest_lock:
            self._latest = None
        self._packet_count = 0
        self._last_sequence = -1
        self._drops = 0
