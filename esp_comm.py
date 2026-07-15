import asyncio
import json
import websockets
import cv2
import serial
import struct
from config import Config
from protocol import ProtocolManager
from transformations import FrameTransformation

class ESPCommunicator:
    def __init__(self):
        self.connected_clients = set()
        self.ser = None
        self.sync_marker = b'\xaa\x55\xa5\x01'
        self.packet_size = 19
        
    def connect_esp(self):
        """Initializes the esp connection."""
        print(f"Opening ESP Port {Config.ESP_PORT} at {Config.BAUD_RATE} baud...")
        self.ser = serial.Serial(Config.ESP_PORT, Config.BAUD_RATE, timeout=0)

    def send_command(self, packet):
        """Thread-safe write to serial. Used by WS and Vision systems."""
        if self.ser and self.ser.is_open:
            self.ser.write(packet)

    def trigger_targeting(self, u_pixel, v_pixel):
        x_man, y_man, z_man = FrameTransformation.getTransformedCoordinates(u_pixel, v_pixel)
        
        # packet = ProtocolManager.pack_coordinate_command(x_man, y_man, z_man)
        print(f"Targeting -> X:{x_man}, Y:{y_man}, Z:{z_man}")
        # self.send_command(packet)

    async def broadcast_ws(self, message_dict):
        """Broadcasts data to all connected web UI clients."""
        if not self.connected_clients:
            return
        message_json = json.dumps(message_dict)
        results = await asyncio.gather(
            *(client.send(message_json) for client in self.connected_clients),
            return_exceptions=True
        )
        for res in results:
            if isinstance(res, Exception):
                print(f"Broadcast warning: {res}")

    async def ws_handler(self, websocket, path):
        """Handles incoming WebSockets commands."""
        self.connected_clients.add(websocket)
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    command = data.get("command", "CMD_STOP")
                    
                    if command == "CMD_MOVE_COORDINATE":
                        packet = ProtocolManager.pack_coordinate_command(
                            data.get("x", 0.0), data.get("y", 0.0), data.get("z", 0.0)
                        )
                    else:
                        packet = ProtocolManager.pack_joint_command(
                            command, data.get("motorId", "T"), 
                            data.get("valA", 0.0), data.get("valB", 0.0), data.get("valC", 0.0)
                        )
                    
                    self.send_command(packet)
                    await self.broadcast_ws({"type": "status", "message": f"Sent {command} to ESP32"})

                except json.JSONDecodeError:
                    print("Invalid JSON received from frontend")
        except websockets.exceptions.ConnectionClosedError:
            pass
        finally:
            if websocket in self.connected_clients:
                self.connected_clients.remove(websocket)

    async def process_serial_loop(self):
        """Reads serial data, separates telemetry from logs, broadcasts to WS."""
        buffer = bytearray()
        text_buffer = ""

        while True:
            if self.ser and self.ser.in_waiting > 0:
                buffer.extend(self.ser.read(self.ser.in_waiting))
                
                while True:
                    sync_idx = buffer.find(self.sync_marker)
                    if sync_idx != -1:
                        if sync_idx > 0:
                            text_buffer += buffer[:sync_idx].decode('ascii', errors='ignore')
                            del buffer[:sync_idx]
                            sync_idx = 0 
                        
                        if len(buffer) >= self.packet_size:
                            packet_bytes = buffer[:self.packet_size]
                            if packet_bytes[-1] == 0x0D:
                                struct_data = packet_bytes[2:18]
                                unpacked = struct.unpack('<BBfffH', struct_data)
                                calc_chk = int(unpacked[2] + unpacked[3] + unpacked[4]) & 0xFFFF
                                
                                await self.broadcast_ws({
                                    "type": "telemetry",
                                    "data": {
                                        "angleA": round(unpacked[2], 2),
                                        "angleB": round(unpacked[3], 2),
                                        "angleC": round(unpacked[4], 2),
                                        "checksum_valid": (calc_chk == unpacked[5])
                                    }
                                })
                            else:
                                text_buffer += chr(buffer[0])
                                del buffer[:1]
                                continue
                                
                            del buffer[:self.packet_size]
                        else:
                            break
                    else:
                        if len(buffer) > 3:
                            text_buffer += buffer[:-3].decode('ascii', errors='ignore')
                            del buffer[:-3]
                        break
                
                while '\n' in text_buffer:
                    line, text_buffer = text_buffer.split('\n', 1)
                    parsed_log = ProtocolManager.parse_esp_log(line)
                    if parsed_log:
                        await self.broadcast_ws(parsed_log)

            await asyncio.sleep(0.01)