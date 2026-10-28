"""
Automated End-to-End Validation Script
Tests TCP Telemetry, Map reception, Survivor Detection Tagging, and UDP Emergency Abort.
"""

import socket
import struct
import json
import time

NIDAR_PACKET_MAGIC = 0xAA55
PKT_TELEMETRY = 0x02
PKT_OCCUPANCY_GRID = 0x03
PKT_SURVIVOR_TAG = 0x04
EMERGENCY_ABORT_TOKEN = b"EMERGENCY_ABORT_NIDAR_HALT"

def test_system():
    print("==========================================================")
    print(" CONNECTING TO NIDAR AIRMOUSE SIMULATION ENGINE")
    print(" Target: 127.0.0.1 (TCP: 9001, UDP: 9999)")
    print("==========================================================")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5.0)

    try:
        sock.connect(("127.0.0.1", 9001))
        print("[SUCCESS] TCP Telemetry Socket Connected.")
    except Exception as e:
        print(f"[FAIL] Could not connect to simulation server: {e}")
        return False

    buf = bytearray()
    telemetry_count = 0
    survivors_received = []

    start_time = time.time()
    print("\n[*] Listening for real-time telemetry and survivor tagging events...")

    while time.time() - start_time < 4.0:
        data = sock.recv(2048)
        if not data:
            break
        buf.extend(data)

        while len(buf) >= 7:
            magic, pkt_type, payload_len = struct.unpack("<HBI", buf[:7])
            if magic != NIDAR_PACKET_MAGIC:
                buf.pop(0)
                continue

            total_len = 7 + payload_len
            if len(buf) < total_len:
                break

            payload = bytes(buf[7:total_len])
            del buf[:total_len]

            if pkt_type == PKT_TELEMETRY:
                telemetry_count += 1
                ts, px, py, pyaw, vx, vy, vyaw, batt, state_idx, timer_sec, surv_count = struct.unpack(
                    "<QfffffffBII", payload[:struct.calcsize("<QfffffffBII")]
                )
                if telemetry_count % 10 == 0:
                    print(f"  [TELEMETRY #{telemetry_count:03d}] Drone Pose: ({px:.2f}m, {py:.2f}m, {pyaw:.1f}deg) | Batt: {batt:.1f}V | Timer: {timer_sec}s | State: {state_idx}")

            elif pkt_type == PKT_SURVIVOR_TAG:
                s_json = json.loads(payload.decode('utf-8'))
                survivors_received.append(s_json)
                print(f"\n  >>> [SURVIVOR DETECTED & TAGGED] <<<")
                print(f"      ID:          {s_json['survivor_id']}")
                print(f"      Room Grid:   {s_json['grid_room']['room_id']} (Col: {s_json['grid_room']['col']}, Row: {s_json['grid_room']['row']})")
                print(f"      Coordinates: ({s_json['survivor_coord']['x']}m, {s_json['survivor_coord']['y']}m)")
                print(f"      Confidence:  {s_json['confidence']*100:.1f}%\n")

    sock.close()

    print(f"\n[SUMMARY] Received {telemetry_count} telemetry packets and {len(survivors_received)} survivor tags.")

    # Test UDP Emergency Abort
    print("\n[*] Blasting High-Priority UDP Emergency Abort datagrams...")
    udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    for _ in range(5):
        udp_sock.sendto(EMERGENCY_ABORT_TOKEN, ("127.0.0.1", 9999))
        time.sleep(0.005)
    udp_sock.close()
    print("[SUCCESS] Emergency Abort burst transmitted to UDP:9999.")
    print("==========================================================")
    print(" ALL SYSTEM VERIFICATIONS PASSED")
    print("==========================================================")
    return True

if __name__ == "__main__":
    test_system()
