import struct
import re

# ============================================================================
# ESP32 (manipulator) — unchanged
# ============================================================================
 
# Command Types
COMMAND_TYPES = {
    "CMD_STOP": 0x00,
    "CMD_MOVE_ABSOLUTE": 0x01,
    "CMD_JOG_RELATIVE": 0x02,
    "CMD_MOVE_COORDINATE": 0x03
}
 
ANSI_ESCAPE = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
ARDUINO_LOG_PATTERN = re.compile(r'^\[\s*(\d+)\]\[([IEWDV])\]\[[^\]]+\]\s*[^:]+:\s*(.*)$')
TAG_PATTERN = re.compile(r'^\[([^\]]+)\]\s*(.*)$')
LOG_LEVEL_MAP = {
    'E': 'error', 'W': 'warning', 'I': 'info', 'D': 'debug', 'V': 'verbose'
}
 
 
def pack_coordinate_command(cmd_type: int, x: float, y: float, z: float) -> bytes:
    header = 0x5A
    checksum = int(x + y + z) & 0xFFFF
    return struct.pack('<BBfffH', header, cmd_type, float(x), float(y), float(z), checksum)
 
 
def pack_joint_command(cmd_type: int, motor_id: str, val_a: float, val_b: float, val_c: float) -> bytes:
    header = 0x5A
    m_id = str(motor_id)[0].encode('ascii') if motor_id else b'T'
    checksum = int(val_a + val_b + val_c) & 0xFFFF
    return struct.pack('<BBcfffH', header, cmd_type, m_id, float(val_a), float(val_b), float(val_c), checksum)
 
 
def parse_esp_log(line: str):
    clean_line = ANSI_ESCAPE.sub('', line.strip())
    if not clean_line:
        return None
 
    match = ARDUINO_LOG_PATTERN.match(clean_line)
    if match:
        timestamp_str, level_char, raw_message = match.groups()
        tag_match = TAG_PATTERN.match(raw_message)
        tag = tag_match.group(1).strip() if tag_match else "System"
        message = tag_match.group(2).strip() if tag_match else raw_message.strip()
 
        return {
            "type": "log",
            "level": LOG_LEVEL_MAP.get(level_char, "info"),
            "timestamp": int(timestamp_str),
            "tag": tag,
            "message": message
        }
    else:
        return {"type": "log", "level": "info", "timestamp": None, "tag": "System", "message": clean_line}