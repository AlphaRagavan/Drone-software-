"""
NIDAR Autonomous AirMouse Challenge
End-to-End Indoor Arena Simulator (15m x 15m Maze with 1m Corridors & 2x2m Rooms)
Simulates companion computer networking, SLAM map streaming, survivor detection, and UDP abort listener.
"""

import socket
import struct
import json
import time
import math
import threading
import sys

NIDAR_PACKET_MAGIC = 0xAA55
PKT_TELEMETRY = 0x02
PKT_OCCUPANCY_GRID = 0x03
PKT_SURVIVOR_TAG = 0x04
PKT_EMERGENCY_ABORT = 0xFF
EMERGENCY_ABORT_TOKEN = b"EMERGENCY_ABORT_NIDAR_HALT"

TCP_PORT = 9001
UDP_ABORT_PORT = 9999

class IndoorDroneSimulator:
    def __init__(self):
        self.running = True
        self.aborted = False
        self.pose_x = 1.0
        self.pose_y = 1.0
        self.pose_yaw = 0.0
        self.step_count = 0
        self.survivors_tagged = set()

        # Simulated survivors placed in modular 2x2m rooms
        self.arena_survivors = [
            {"id": "SURVIVOR_ALPHA", "x": 4.88, "y": 7.42, "conf": 0.94},
            {"id": "SURVIVOR_BRAVO", "x": 10.45, "y": 3.20, "conf": 0.89},
            {"id": "SURVIVOR_CHARLIE", "x": 13.10, "y": 12.80, "conf": 0.91}
        ]

        self.start_abort_listener()
        self.start_tcp_server()

    def start_abort_listener(self):
        def listener():
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("0.0.0.0", UDP_ABORT_PORT))
            print(f"[SIMULATOR] UDP Abort listener active on port {UDP_ABORT_PORT}", flush=True)

            while self.running:
                data, addr = sock.recvfrom(512)
                if EMERGENCY_ABORT_TOKEN in data:
                    self.aborted = True
                    print(f"\n========================================================", flush=True)
                    print(f"[SIMULATOR CRITICAL] EMERGENCY ABORT RECEIVED FROM {addr}!", flush=True)
                    print(f"[SIMULATOR] ALL DRONE THRUST HARD DISARMED INSTANTLY.", flush=True)
                    print(f"========================================================\n", flush=True)

        threading.Thread(target=listener, daemon=True).start()

    def start_tcp_server(self):
        def server():
            srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            srv.bind(("0.0.0.0", TCP_PORT))
            srv.listen(5)
            print(f"[SIMULATOR] TCP Telemetry & Map Server listening on port {TCP_PORT}", flush=True)

            while self.running:
                client_sock, addr = srv.accept()
                print(f"[SIMULATOR] GCS Connected from {addr}", flush=True)
                # Reset abort on fresh client connection for interactive testing
                self.aborted = False
                threading.Thread(target=self.client_handler, args=(client_sock,), daemon=True).start()

        threading.Thread(target=server, daemon=True).start()

    def client_handler(self, sock):
        start_time = time.time()
        try:
            while self.running and not self.aborted:
                self.step_count += 1
                t = self.step_count * 0.1

                # Simulate drone navigating along 1m corridor through rooms
                self.pose_x = 1.0 + 5.0 * (1.0 - math.cos(t * 0.2)) + 0.1 * math.sin(t)
                self.pose_y = 1.0 + 4.5 * math.sin(t * 0.2) + 0.05 * math.cos(t)
                self.pose_yaw = (t * 20.0) % 360.0

                elapsed_sec = int(time.time() - start_time)

                # 1. Telemetry Frame (timestamp, pose_x, pose_y, pose_yaw, vel_x, vel_y, vel_yaw, batt, state, timer, survivors)
                telem_payload = struct.pack(
                    "<QfffffffBII",
                    int(time.time() * 1000),
                    self.pose_x, self.pose_y, self.pose_yaw,
                    0.45, 0.0, 0.2, 15.6,
                    2, # STATE_EXPLORE_MAZE
                    elapsed_sec,
                    len(self.survivors_tagged)
                )
                hdr = struct.pack("<HBI", NIDAR_PACKET_MAGIC, PKT_TELEMETRY, len(telem_payload))
                sock.sendall(hdr + telem_payload)

                # 2. Check for nearby survivors (Detection within 5.0m for immediate simulation demo)
                for surv in self.arena_survivors:
                    if surv["id"] not in self.survivors_tagged:
                        dist = math.hypot(surv["x"] - self.pose_x, surv["y"] - self.pose_y)
                        if dist < 6.0:
                            self.survivors_tagged.add(surv["id"])
                            col = int(surv["x"] // 2.0)
                            row = int(surv["y"] // 2.0)
                            room_id = f"Room_{col}_{row}"

                            payload_dict = {
                                "type": "SURVIVOR_TAGGED",
                                "timestamp_ms": int(time.time() * 1000),
                                "survivor_id": surv["id"],
                                "confidence": surv["conf"],
                                "bounding_box": {"x": 240, "y": 160, "w": 180, "h": 280},
                                "drone_pose": {"x": round(self.pose_x, 2), "y": round(self.pose_y, 2), "yaw_deg": round(self.pose_yaw, 1)},
                                "survivor_coord": {"x": surv["x"], "y": surv["y"]},
                                "grid_room": {"room_id": room_id, "col": col, "row": row, "room_bounds": [col*2.0, row*2.0, (col+1)*2.0, (row+1)*2.0]}
                            }
                            json_bytes = json.dumps(payload_dict).encode('utf-8')
                            tag_hdr = struct.pack("<HBI", NIDAR_PACKET_MAGIC, PKT_SURVIVOR_TAG, len(json_bytes))
                            sock.sendall(tag_hdr + json_bytes)
                            print(f"[SIMULATOR] Broadcasted {surv['id']} in {room_id}", flush=True)

                time.sleep(0.05) # 20 Hz loop
        except Exception as e:
            print(f"[SIMULATOR] Client disconnected: {e}", flush=True)
        finally:
            sock.close()

def main():
    print("==========================================================", flush=True)
    print(" NIDAR AIRMOUSE 15m x 15m INDOOR SIMULATION SERVER", flush=True)
    print("==========================================================", flush=True)
    sim = IndoorDroneSimulator()
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    print("[SIMULATOR] Simulation ended.", flush=True)

if __name__ == "__main__":
    main()
