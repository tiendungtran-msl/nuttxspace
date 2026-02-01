"""
UAV Ground Control Station - Serial Receiver

Thread riêng cho việc nhận dữ liệu serial.
Non-blocking, emit Qt signals khi có packet hợp lệ.
"""

import time
import threading
from typing import Optional, Callable
from enum import Enum, auto

import serial
from PySide6.QtCore import QObject, Signal

from .packet import (
    TelemetryData, decode_packet,
    TELEM_PACKET_SIZE, TELEM_MAGIC_START, TELEM_MAGIC_END
)


class DecoderState(Enum):
    """State machine states for packet synchronization"""
    SYNC_1 = auto()      # Waiting for first magic byte (0x55)
    SYNC_2 = auto()      # Waiting for second magic byte (0xAA)
    PAYLOAD = auto()     # Reading payload


class PacketDecoder:
    """
    State machine decoder for telemetry packets.
    
    Xử lý byte-by-byte để sync và decode packets.
    """
    
    def __init__(self):
        self.state = DecoderState.SYNC_1
        self.buffer = bytearray()
        self.packets_decoded = 0
        self.sync_errors = 0
        self.crc_errors = 0
        self.last_sequence = -1
        self.sequence_drops = 0
    
    def reset(self):
        """Reset decoder state"""
        self.state = DecoderState.SYNC_1
        self.buffer.clear()
    
    def feed(self, data: bytes) -> list[TelemetryData]:
        """
        Feed raw bytes to decoder.
        
        Returns list of decoded packets (usually 0 or 1).
        """
        packets = []
        
        for byte in data:
            result = self._process_byte(byte)
            if result is not None:
                packets.append(result)
        
        return packets
    
    def _process_byte(self, byte: int) -> Optional[TelemetryData]:
        """Process single byte through state machine"""
        
        if self.state == DecoderState.SYNC_1:
            # Looking for 0xAA (first byte of 0x55AA in little-endian on wire)
            if byte == 0xAA:
                self.buffer.clear()
                self.buffer.append(byte)
                self.state = DecoderState.SYNC_2
            return None
        
        elif self.state == DecoderState.SYNC_2:
            # Looking for 0x55 (second byte of 0x55AA in little-endian on wire)
            if byte == 0x55:
                self.buffer.append(byte)
                self.state = DecoderState.PAYLOAD
            else:
                # Not magic, check if this could be new 0xAA
                self.sync_errors += 1
                if byte == 0xAA:
                    self.buffer.clear()
                    self.buffer.append(byte)
                    self.state = DecoderState.SYNC_2
                else:
                    self.state = DecoderState.SYNC_1
            return None
        
        elif self.state == DecoderState.PAYLOAD:
            self.buffer.append(byte)
            
            if len(self.buffer) >= TELEM_PACKET_SIZE:
                # Full packet received
                packet = self._decode_buffer()
                self.state = DecoderState.SYNC_1
                return packet
            
            return None
        
        return None
    
    def _decode_buffer(self) -> Optional[TelemetryData]:
        """Decode complete buffer"""
        raw = bytes(self.buffer)
        self.buffer.clear()
        
        data = decode_packet(raw)
        
        if data is None:
            self.sync_errors += 1
            return None
        
        if not data.crc_valid:
            self.crc_errors += 1
            return None
        
        # Check sequence continuity
        if self.last_sequence >= 0:
            expected = (self.last_sequence + 1) & 0xFFFF
            if data.sequence != expected:
                # Sequence gap detected
                gap = (data.sequence - expected) & 0xFFFF
                self.sequence_drops += gap
        
        self.last_sequence = data.sequence
        self.packets_decoded += 1
        
        # Add PC timestamp
        data.pc_timestamp_ms = int(time.time() * 1000)
        
        return data


class SerialReceiver(QObject):
    """
    Serial receiver running in dedicated thread.
    
    Emits Qt signals for thread-safe communication with GUI.
    """
    
    # Signals
    packet_received = Signal(object)  # TelemetryData
    connection_changed = Signal(bool)  # connected/disconnected
    error_occurred = Signal(str)       # error message
    stats_updated = Signal(dict)       # statistics
    
    def __init__(self, parent=None):
        super().__init__(parent)
        
        self.serial: Optional[serial.Serial] = None
        self.decoder = PacketDecoder()
        
        self._running = False
        self._thread: Optional[threading.Thread] = None
        
        # Statistics
        self.bytes_received = 0
        self.packets_received = 0
        self.connect_time = 0
    
    def connect(self, port: str, baudrate: int = 921600) -> bool:
        """
        Open serial port and start receiver thread.
        """
        if self._running:
            return False
        
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1,  # 100ms timeout for non-blocking
                write_timeout=0.1
            )
            
            # Reset stats
            self.bytes_received = 0
            self.packets_received = 0
            self.connect_time = time.time()
            self.decoder.reset()
            
            # Start receiver thread
            self._running = True
            self._thread = threading.Thread(target=self._receive_loop, daemon=True)
            self._thread.start()
            
            self.connection_changed.emit(True)
            return True
            
        except serial.SerialException as e:
            self.error_occurred.emit(f"Failed to open {port}: {e}")
            return False
    
    def disconnect(self):
        """
        Stop receiver and close serial port.
        """
        self._running = False
        
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        
        if self.serial is not None:
            try:
                self.serial.close()
            except:
                pass
            self.serial = None
        
        self.connection_changed.emit(False)
    
    def is_connected(self) -> bool:
        """Check if connected"""
        return self._running and self.serial is not None and self.serial.is_open
    
    def _receive_loop(self):
        """
        Main receive loop (runs in dedicated thread).
        """
        last_stats_time = time.time()
        
        while self._running and self.serial is not None:
            try:
                # Read available data
                if self.serial.in_waiting > 0:
                    data = self.serial.read(self.serial.in_waiting)
                    self.bytes_received += len(data)
                    
                    # Decode packets
                    packets = self.decoder.feed(data)
                    
                    for packet in packets:
                        self.packets_received += 1
                        self.packet_received.emit(packet)
                
                # Update stats periodically (every 500ms)
                now = time.time()
                if now - last_stats_time >= 0.5:
                    self._emit_stats()
                    last_stats_time = now
                
                # Small sleep to prevent busy-loop
                time.sleep(0.001)  # 1ms
                
            except serial.SerialException as e:
                self.error_occurred.emit(f"Serial error: {e}")
                break
            except Exception as e:
                self.error_occurred.emit(f"Receive error: {e}")
                break
        
        # Cleanup
        self._running = False
        self.connection_changed.emit(False)
    
    def _emit_stats(self):
        """Emit current statistics"""
        elapsed = time.time() - self.connect_time
        
        stats = {
            'bytes_received': self.bytes_received,
            'packets_received': self.packets_received,
            'packets_decoded': self.decoder.packets_decoded,
            'sync_errors': self.decoder.sync_errors,
            'crc_errors': self.decoder.crc_errors,
            'sequence_drops': self.decoder.sequence_drops,
            'elapsed_seconds': elapsed,
            'bytes_per_second': self.bytes_received / max(elapsed, 0.001),
            'packets_per_second': self.packets_received / max(elapsed, 0.001)
        }
        
        self.stats_updated.emit(stats)
