import asyncio
import threading
import websockets
import json
from config import Config
from esp_comm import ESPCommunicator
from arduino_comm import ArduinoCommunicator
from vision import VisionSystem
from protocol import ProtocolManager

async def main():
    # Grab the active event loop so we can pass things to it from threads
    loop = asyncio.get_running_loop()

    # 1. Initialize Communications
    esp_comms = ESPCommunicator()
    arduino_comms = ArduinoCommunicator()
    
    try:
        esp_comms.connect_esp()
    except Exception as e:
        print(f"Failed to open ESP port: {e}")

    try:
        arduino_comms.connect_arduino()
    except Exception as e:
        print(f"Failed to open Arduino port: {e}")

    # --- Thread-Safe Frontend Broadcaster ---
    def send_targets_to_frontend(targets):
        """ This runs in the Vision Thread but safely triggers an async broadcast """
        payload = {
            "type": "target_locations",
            "targets": targets # e.g. [{"x": 100, "y": 200, "conf": 0.85}, ...]
        }
        # Schedule the coroutine safely in the main asyncio loop
        asyncio.run_coroutine_threadsafe(esp_comms.broadcast_ws(payload), loop)

    # 2. Initialize Vision System with BOTH callbacks
    vision = VisionSystem(
        target_detected_callback=esp_comms.trigger_targeting,
        detections_callback=send_targets_to_frontend
    )
    vision.setup()

    # 3. Start Vision in a separate background thread
    vision_thread = threading.Thread(target=vision.run, daemon=True)
    vision_thread.start()

    # --- Central WebSocket Handler ---
    async def central_ws_handler(websocket, path):
        esp_comms.connected_clients.add(websocket)
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    target = data.get("target", "esp") # Default to ESP if not specified
                    command = data.get("command", "")

                    if target == "arduino":
                        # Route to Mobile Base (e.g., 'f', 'b', 's')
                        arduino_comms.send_command(command)
                        await esp_comms.broadcast_ws({"type": "status", "message": f"Sent '{command}' to Arduino"})
                    
                    elif target == "esp":
                        # Route to Delta Manipulator
                        if command == "CMD_MOVE_COORDINATE":
                            packet = ProtocolManager.pack_coordinate_command(
                                data.get("x", 0.0), data.get("y", 0.0), data.get("z", 0.0)
                            )
                        else:
                            packet = ProtocolManager.pack_joint_command(
                                command, data.get("motorId", "T"), 
                                data.get("valA", 0.0), data.get("valB", 0.0), data.get("valC", 0.0)
                            )
                        esp_comms.send_command(packet)
                        await esp_comms.broadcast_ws({"type": "status", "message": f"Sent {command} to ESP32"})

                except json.JSONDecodeError:
                    print("Invalid JSON received from frontend")
        except websockets.exceptions.ConnectionClosedError:
            pass
        finally:
            if websocket in esp_comms.connected_clients:
                esp_comms.connected_clients.remove(websocket)

    # 4. Start WebSocket server using the new central handler
    ws_server = await websockets.serve(
        central_ws_handler, 
        Config.WS_HOST, 
        Config.WS_PORT
    )
    print(f"WebSocket server running on ws://{Config.WS_HOST}:{Config.WS_PORT}")

    # 5. Run BOTH Serial processing loops concurrently using asyncio.gather
    try:
        await asyncio.gather(
            esp_comms.process_serial_loop(),
            arduino_comms.process_serial_loop(broadcast_callback=esp_comms.broadcast_ws)
        )
    except asyncio.CancelledError:
        pass
    finally:
        vision.stop()
        vision_thread.join(timeout=2.0)

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        print("\nShutting down gracefully.")
    finally:
        loop.close()