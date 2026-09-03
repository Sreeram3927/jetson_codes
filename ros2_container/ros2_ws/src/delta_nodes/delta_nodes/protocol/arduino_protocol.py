import struct
import json

# ============================================================================
# Arduino (mobile base) — binary protocol, mirrors firmware protocol.h/.cpp
# ============================================================================
 
ARDUINO_SYNC0 = 0xAA
ARDUINO_SYNC1 = 0x56
 
# Must match PROTOCOL_VERSION_MAJOR/MINOR in the firmware's protocol.h.
# Bump the major number here (and in firmware) on breaking frame-layout
# changes; minor for backward-compatible additions.
ARDUINO_PROTOCOL_VERSION = (1, 0)
 
ARDUINO_CMD = {
    "DRIVE": 0x01,          # int8 linear_pct, int8 angular_pct   (-100..100)
    "SOFT_STOP": 0x02,      # (no payload)
    "SET_LASER": 0x03,      # uint8 power_pct                     (0..100)
    "SET_MAX_SPEED": 0x04,  # uint8 max_speed_pct                 (0..100)
    "SET_RAMP_RATE": 0x05,  # uint8 accel_pct, uint8 decel_pct
    "GET_VERSION": 0x06,    # (no payload)
}
 
ARDUINO_TELEM_STATE = 0xA1    # int8 linear_actual, int8 angular_actual, uint8 laser_pct, uint8 estop_sensed, uint16 uptime_ms (LE)
ARDUINO_TELEM_VERSION = 0xA2  # uint8 major, uint8 minor
 
 
def _arduino_checksum(type_byte: int, payload: bytes) -> int:
    c = type_byte ^ len(payload)
    for b in payload:
        c ^= b
    return c & 0xFF
 
 
def _pack_arduino_frame(type_byte: int, payload: bytes = b"") -> bytes:
    checksum = _arduino_checksum(type_byte, payload)
    return bytes([ARDUINO_SYNC0, ARDUINO_SYNC1, type_byte, len(payload)]) + payload + bytes([checksum])
 
 
def pack_arduino_drive(linear_pct: int, angular_pct: int) -> bytes:
    linear_pct = max(-100, min(100, int(round(linear_pct))))
    angular_pct = max(-100, min(100, int(round(angular_pct))))
    payload = struct.pack('<bb', linear_pct, angular_pct)
    return _pack_arduino_frame(ARDUINO_CMD["DRIVE"], payload)
 
 
def pack_arduino_soft_stop() -> bytes:
    return _pack_arduino_frame(ARDUINO_CMD["SOFT_STOP"])
 
 
def pack_arduino_set_laser(power_pct: int) -> bytes:
    power_pct = max(0, min(100, int(round(power_pct))))
    payload = struct.pack('<B', power_pct)
    return _pack_arduino_frame(ARDUINO_CMD["SET_LASER"], payload)
 
 
def pack_arduino_set_max_speed(max_speed_pct: int) -> bytes:
    max_speed_pct = max(0, min(100, int(round(max_speed_pct))))
    payload = struct.pack('<B', max_speed_pct)
    return _pack_arduino_frame(ARDUINO_CMD["SET_MAX_SPEED"], payload)
 
 
def pack_arduino_set_ramp_rate(accel_pct: int, decel_pct: int) -> bytes:
    accel_pct = max(1, min(100, int(round(accel_pct))))
    decel_pct = max(1, min(100, int(round(decel_pct))))
    payload = struct.pack('<BB', accel_pct, decel_pct)
    return _pack_arduino_frame(ARDUINO_CMD["SET_RAMP_RATE"], payload)
 
 
def pack_arduino_get_version() -> bytes:
    return _pack_arduino_frame(ARDUINO_CMD["GET_VERSION"])

def parse_arduino_log(line: str):
    clean_line = line.strip()
    if not clean_line:
        return None
    
    try:
        data = json.loads(line)
    except (json.JSONDecodeError, TypeError):
        return None

    # Make sure this is actually an Arduino log
    if not isinstance(data, dict):
        return None

    if data.get("type") != "log":
        return None
    
    return {
        "type": "log",
        "level": str(data.get("level", "info")),
        "timestamp": data.get("timestamp"),
        "tag": str(data.get("tag", "System")),
        "message": str(data.get("message", ""))
    }


class ArduinoLinkParser:
    """
    Stateful demuxer for the Arduino's shared serial stream (binary telemetry
    frames + JSON log lines, interleaved). Feed it raw bytes as they arrive;
    it returns a list of parsed event dicts, each tagged by "type":
 
        {"type": "telemetry", "linear_actual": int, "angular_actual": int,
         "laser_pct": int, "estop_sensed": bool, "uptime_ms": int}
        {"type": "version", "major": int, "minor": int}
        {"type": "log", "level": ..., "timestamp": ..., "tag": ..., "message": ...}
 
    Bytes that don't yet resolve into a complete frame or line are buffered
    internally until more data arrives on the next feed() call. Frames with
    a bad checksum are silently dropped (matching the firmware's own
    resync-on-next-SYNC behavior).
 
    IMPORTANT: feed this raw bytes from serial, not a utf-8-decoded string —
    decoding before demuxing corrupts binary frame bytes that aren't valid
    UTF-8 and desyncs the framing.
    """
 
    _WAIT_SYNC0, _WAIT_SYNC1, _WAIT_TYPE, _WAIT_LEN, _WAIT_PAYLOAD, _WAIT_CHECKSUM = range(6)
 
    def __init__(self):
        self._text_buf = bytearray()
        self._state = self._WAIT_SYNC0
        self._frame_type = 0
        self._frame_len = 0
        self._payload = bytearray()
 
    def feed(self, data: bytes):
        events = []
        for byte in data:
            self._feed_byte(byte, events)
        return events
 
    def _feed_byte(self, byte: int, events: list):
        if self._state == self._WAIT_SYNC0:
            if byte == ARDUINO_SYNC0:
                self._flush_text(events)
                self._state = self._WAIT_SYNC1
            elif byte == 0x0A:  # '\n'
                self._text_buf.append(byte)
                self._flush_text(events)
            else:
                self._text_buf.append(byte)
            return
 
        if self._state == self._WAIT_SYNC1:
            if byte == ARDUINO_SYNC1:
                self._state = self._WAIT_TYPE
            else:
                # False alarm - 0xAA wasn't actually a sync byte. Put both
                # bytes back into the text buffer (they'll be dropped by
                # errors='ignore' if genuinely not valid text) and resume.
                self._text_buf.append(ARDUINO_SYNC0)
                self._text_buf.append(byte)
                self._state = self._WAIT_SYNC0
            return
 
        if self._state == self._WAIT_TYPE:
            self._frame_type = byte
            self._state = self._WAIT_LEN
            return
 
        if self._state == self._WAIT_LEN:
            self._frame_len = byte
            self._payload = bytearray()
            self._state = self._WAIT_CHECKSUM if self._frame_len == 0 else self._WAIT_PAYLOAD
            return
 
        if self._state == self._WAIT_PAYLOAD:
            self._payload.append(byte)
            if len(self._payload) >= self._frame_len:
                self._state = self._WAIT_CHECKSUM
            return
 
        if self._state == self._WAIT_CHECKSUM:
            expected = _arduino_checksum(self._frame_type, bytes(self._payload))
            if byte == expected:
                parsed = self._parse_frame(self._frame_type, bytes(self._payload))
                if parsed:
                    events.append(parsed)
            self._state = self._WAIT_SYNC0
            return
 
    def _flush_text(self, events: list):
        if not self._text_buf:
            return
        line = bytes(self._text_buf).decode('utf-8', errors='ignore').strip()
        self._text_buf = bytearray()
        if line:
            parsed = parse_arduino_log(line)
            if parsed:
                events.append(parsed)
 
    @staticmethod
    def _parse_frame(frame_type: int, payload: bytes):
        if frame_type == ARDUINO_TELEM_STATE and len(payload) == 6:
            linear, angular, laser_pct, estop, uptime = struct.unpack('<bbBBH', payload)
            return {
                "type": "telemetry",
                "linear_actual": linear,
                "angular_actual": angular,
                "laser_pct": laser_pct,
                "estop_sensed": bool(estop),
                "uptime_ms": uptime,
            }
        if frame_type == ARDUINO_TELEM_VERSION and len(payload) == 2:
            major, minor = struct.unpack('<BB', payload)
            return {"type": "version", "major": major, "minor": minor}
        return None

