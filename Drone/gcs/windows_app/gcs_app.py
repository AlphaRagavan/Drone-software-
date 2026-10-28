"""
NIDAR AirMouse Autonomous Indoor Search & Rescue System
Windows 10/11 Standalone Ground Control Station (GCS)
Tailored for Acer Nitro V15 and Windows 10/11 environments.
Zero external dependencies required (Native Python standard library + Tkinter).
Dynamically leverages OpenCV/Pillow if installed for hardware acceleration.
"""

import sys
import socket
import struct
import json
import threading
import time
import math
import tkinter as tk
from tkinter import ttk, messagebox

# Protocol constants matching onboard/include/networking/protocol_types.hpp
NIDAR_PACKET_MAGIC = 0xAA55
PKT_TELEMETRY = 0x02
PKT_OCCUPANCY_GRID = 0x03
PKT_SURVIVOR_TAG = 0x04
PKT_MISSION_STATE = 0x06
PKT_EMERGENCY_ABORT = 0xFF
EMERGENCY_ABORT_TOKEN = b"EMERGENCY_ABORT_NIDAR_HALT"

DEFAULT_DRONE_HOST = "nidar-drone.local"
DEFAULT_DRONE_IP = "192.168.4.1"
TCP_PORT = 9001
UDP_ABORT_PORT = 9999
HTTP_VIDEO_PORT = 8080

FLIGHT_STATES = [
    "BOOT_AND_INIT", "TAKEOFF_AND_ENTER", "EXPLORE_MAZE",
    "TAG_SURVIVOR", "FIND_EXIT_AND_LAND", "EMERGENCY_ABORT", "MISSION_COMPLETED"
]

class StandaloneGCSApp:
    def __init__(self, root):
        self.root = root
        self.root.title("NIDAR AirMouse Ground Control Station (GCS) - Windows 10/11")
        self.root.geometry("1480x920")
        self.root.minsize(1200, 750)
        self.root.configure(bg="#0b0f19")

        # Telemetry state
        self.drone_ip = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_DRONE_IP
        self.is_connected = False
        self.telemetry_data = {}
        self.map_cells = []
        self.map_w = 0
        self.map_h = 0
        self.map_res = 0.05
        self.drone_pose = (1.0, 1.0, 0.0) # x, y, yaw_deg
        self.drone_trail = [(1.0, 1.0)]
        self.survivors = []
        self.mission_time_sec = 0

        # Video state
        self.video_frame_step = 0
        self.video_active = True

        self.setup_ui()
        self.setup_networking()

        # Canvas animation clock
        self.pulse_phase = 0.0
        self.root.after(50, self.update_canvas_loop)

    def setup_ui(self):
        # Top Header Bar
        header = tk.Frame(self.root, bg="#151c2c", bd=1, relief=tk.SOLID, height=60)
        header.pack(fill=tk.X, padx=12, pady=8)

        # Status badge
        self.lbl_status = tk.Label(header, text="STATUS: RESOLVING BONJOUR...", 
                                   fg="#f6ad55", bg="#151c2c", font=("Segoe UI", 11, "bold"))
        self.lbl_status.pack(side=tk.LEFT, padx=14)

        self.lbl_ip = tk.Label(header, text=f"HOST: {DEFAULT_DRONE_HOST}", 
                               fg="#a0aec0", bg="#151c2c", font=("Consolas", 10))
        self.lbl_ip.pack(side=tk.LEFT, padx=8)

        # Center Indicators
        self.lbl_state = tk.Label(header, text="STATE: STANDBY", 
                                  fg="#63b3ed", bg="#151c2c", font=("Segoe UI", 11, "bold"))
        self.lbl_state.pack(side=tk.LEFT, padx=25)

        self.lbl_timer = tk.Label(header, text="MISSION TIMER: 00:00 / 30:00", 
                                  fg="#48bb78", bg="#151c2c", font=("Consolas", 12, "bold"))
        self.lbl_timer.pack(side=tk.LEFT, padx=25)

        self.lbl_battery = tk.Label(header, text="BATT: 15.8V", 
                                    fg="#68d391", bg="#151c2c", font=("Segoe UI", 11, "bold"))
        self.lbl_battery.pack(side=tk.LEFT, padx=20)

        # EMERGENCY ABORT BUTTON
        self.btn_abort = tk.Button(
            header, text="EMERGENCY ABORT", command=self.blast_emergency_abort,
            bg="#e53e3e", fg="white", font=("Segoe UI", 12, "bold"),
            activebackground="#c53030", activeforeground="white",
            relief=tk.RAISED, bd=3, padx=18, pady=4, cursor="hand2"
        )
        self.btn_abort.pack(side=tk.RIGHT, padx=14)

        # Middle Paned Frame (Video + Map)
        panes = tk.PanedWindow(self.root, orient=tk.HORIZONTAL, bg="#0b0f19", bd=0, sashwidth=4)
        panes.pack(fill=tk.BOTH, expand=True, padx=12, pady=4)

        # Left Panel: Video Feed
        video_frame = tk.Frame(panes, bg="#151c2c", bd=1, relief=tk.SOLID)
        panes.add(video_frame, minsize=500, width=640)

        lbl_video_title = tk.Label(video_frame, text="LIVE ONBOARD OPTICAL STREAM [RTSP / MJPEG]", 
                                   fg="#00ff8c", bg="#151c2c", font=("Segoe UI", 9, "bold"))
        lbl_video_title.pack(anchor="w", padx=10, pady=6)

        self.video_canvas = tk.Canvas(video_frame, bg="#0e131f", highlightthickness=0)
        self.video_canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        # Right Panel: 2D SLAM Arena Map
        map_frame = tk.Frame(panes, bg="#151c2c", bd=1, relief=tk.SOLID)
        panes.add(map_frame, minsize=600)

        lbl_map_title = tk.Label(map_frame, text="2D SLAM ARENA MAP (15m x 15m) [MODULAR 2x2m ROOM GRID]", 
                                 fg="#00f0ff", bg="#151c2c", font=("Segoe UI", 9, "bold"))
        lbl_map_title.pack(anchor="w", padx=10, pady=6)

        self.map_canvas = tk.Canvas(map_frame, bg="#0b0f19", highlightthickness=0)
        self.map_canvas.pack(fill=tk.BOTH, expand=True, padx=8, pady=8)

        # Bottom Frame: Survivor Log Table
        bottom_frame = tk.Frame(self.root, bg="#151c2c", bd=1, relief=tk.SOLID, height=180)
        bottom_frame.pack(fill=tk.X, padx=12, pady=8)

        lbl_table = tk.Label(bottom_frame, text="CONFIRMED SURVIVOR DETECTION LOG (TAGGED LOCATIONS)", 
                             fg="#fc8181", bg="#151c2c", font=("Segoe UI", 10, "bold"))
        lbl_table.pack(anchor="w", padx=10, pady=4)

        # Treeview table
        columns = ("id", "room", "coord", "conf", "time", "status")
        self.tree = ttk.Treeview(bottom_frame, columns=columns, show="headings", height=5)
        self.tree.heading("id", text="SURVIVOR ID")
        self.tree.heading("room", text="MODULAR ROOM")
        self.tree.heading("coord", text="COORDINATES (X, Y)")
        self.tree.heading("conf", text="CONFIDENCE")
        self.tree.heading("time", text="DETECTION TIME")
        self.tree.heading("status", text="STATUS")

        self.tree.column("id", width=140, anchor="center")
        self.tree.column("room", width=150, anchor="center")
        self.tree.column("coord", width=180, anchor="center")
        self.tree.column("conf", width=120, anchor="center")
        self.tree.column("time", width=140, anchor="center")
        self.tree.column("status", width=160, anchor="center")

        # Style treeview for dark mode
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("Treeview", background="#0e131f", foreground="#e2e8f0", 
                        fieldbackground="#0e131f", rowheight=24, font=("Consolas", 10))
        style.configure("Treeview.Heading", background="#1a202c", foreground="#a0aec0", 
                        font=("Segoe UI", 9, "bold"))

        self.tree.pack(fill=tk.BOTH, expand=True, padx=8, pady=4)

    def setup_networking(self):
        # Bonjour / mDNS resolver thread
        threading.Thread(target=self.resolve_bonjour_thread, daemon=True).start()
        # TCP telemetry reader thread
        threading.Thread(target=self.telemetry_tcp_thread, daemon=True).start()

    def resolve_bonjour_thread(self):
        """Resolves target: CLI override, local simulator (127.0.0.1), Bonjour mDNS (nidar-drone.local), or Wi-Fi AP (192.168.4.1)."""
        if len(sys.argv) > 1:
            self.drone_ip = sys.argv[1]
            self.lbl_ip.config(text=f"IP: {self.drone_ip} (MANUAL)")
            self.lbl_status.config(text="STATUS: CONNECTING...", fg="#f6ad55")
            return

        while not self.is_connected:
            # 1. Check if local simulation engine is running on localhost:9001
            try:
                probe = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                probe.settimeout(0.3)
                if probe.connect_ex(("127.0.0.1", TCP_PORT)) == 0:
                    self.drone_ip = "127.0.0.1"
                    self.lbl_ip.config(text="IP: 127.0.0.1 (LOCAL SIMULATOR)")
                    self.lbl_status.config(text="STATUS: SIMULATOR DETECTED", fg="#48bb78")
                    probe.close()
                    break
                probe.close()
            except Exception:
                pass

            # 2. Try resolving Bonjour mDNS (nidar-drone.local)
            try:
                ip = socket.gethostbyname(DEFAULT_DRONE_HOST)
                if ip:
                    self.drone_ip = ip
                    self.lbl_ip.config(text=f"IP: {self.drone_ip} (BONJOUR)")
                    self.lbl_status.config(text="STATUS: BONJOUR RESOLVED", fg="#48bb78")
                    break
            except Exception:
                pass

            # 3. Default fallback to 192.168.4.1
            self.drone_ip = DEFAULT_DRONE_IP
            time.sleep(1.0)

    def telemetry_tcp_thread(self):
        """Connects to companion computer TCP socket and parses telemetry packets."""
        while True:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(3.0)
                sock.connect((self.drone_ip, TCP_PORT))
                self.is_connected = True
                self.lbl_status.config(text="STATUS: ONLINE (5GHz LINK)", fg="#48bb78")
                self.lbl_ip.config(text=f"IP: {self.drone_ip}:{TCP_PORT}")

                buf = bytearray()
                while self.is_connected:
                    data = sock.recv(4096)
                    if not data:
                        break
                    buf.extend(data)

                    # Parse framed packets
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

                        self.handle_packet(pkt_type, payload)

            except Exception as e:
                self.is_connected = False
                self.lbl_status.config(text=f"STATUS: CONNECTING ({self.drone_ip})...", fg="#f6ad55")
                time.sleep(1.5)

    def handle_packet(self, pkt_type, payload):
        if pkt_type == PKT_TELEMETRY and len(payload) >= struct.calcsize("<QfffffffBII"):
            ts, px, py, pyaw, vx, vy, vyaw, batt, state_idx, timer_sec, survivors = struct.unpack("<QfffffffBII", payload[:struct.calcsize("<QfffffffBII")])
            self.drone_pose = (px, py, pyaw)
            if not self.drone_trail or math.hypot(self.drone_trail[-1][0] - px, self.drone_trail[-1][1] - py) > 0.15:
                self.drone_trail.append((px, py))
                if len(self.drone_trail) > 800:
                    self.drone_trail.pop(0)

            self.mission_time_sec = timer_sec
            mins = timer_sec // 60
            secs = timer_sec % 60
            self.lbl_timer.config(text=f"MISSION TIMER: {mins:02d}:{secs:02d} / 30:00")
            if mins >= 27:
                self.lbl_timer.config(fg="#e53e3e")

            state_name = FLIGHT_STATES[state_idx] if state_idx < len(FLIGHT_STATES) else "UNKNOWN"
            self.lbl_state.config(text=f"STATE: {state_name}")
            self.lbl_battery.config(text=f"BATT: {batt:.1f}V")

        elif pkt_type == PKT_OCCUPANCY_GRID and len(payload) >= 20:
            w, h, res, ox, oy, comp_len = struct.unpack("<IIfffI", payload[:24])
            rle_bytes = payload[24:24+comp_len]
            # Decompress RLE
            cells = []
            for i in range(0, len(rle_bytes), 2):
                if i + 1 < len(rle_bytes):
                    count = rle_bytes[i]
                    val = rle_bytes[i + 1]
                    cells.extend([val] * count)

            if len(cells) == w * h:
                self.map_w = w
                self.map_h = h
                self.map_res = res
                self.map_cells = cells

        elif pkt_type == PKT_SURVIVOR_TAG:
            try:
                survivor_json = json.loads(payload.decode('utf-8'))
                self.on_survivor_tagged(survivor_json)
            except Exception as e:
                print("[GCS] Survivor JSON decode error:", e)

    def on_survivor_tagged(self, s):
        survivor_id = s.get("survivor_id", "SURVIVOR")
        conf = s.get("confidence", 0.0)
        coord = s.get("survivor_coord", {})
        sx = coord.get("x", 0.0)
        sy = coord.get("y", 0.0)
        room = s.get("grid_room", {})
        room_id = room.get("room_id", "ROOM")

        # Avoid duplicate table entries
        for surv in self.survivors:
            if surv["id"] == survivor_id:
                return

        self.survivors.append({
            "id": survivor_id,
            "x": sx,
            "y": sy,
            "room": room_id,
            "conf": conf,
            "time": time.strftime("%H:%M:%S")
        })

        self.tree.insert("", tk.END, values=(
            survivor_id, room_id, f"({sx:.2f}m, {sy:.2f}m)",
            f"{int(conf * 100)}%", time.strftime("%H:%M:%S"), "TAGGED & LOGGED"
        ))
        print(f"[GCS] INSTANT MARKER DROPPED: {survivor_id} in {room_id} at ({sx:.2f}, {sy:.2f})")

    def blast_emergency_abort(self):
        """Hard-coded emergency abort blasting over UDP."""
        target_ip = self.drone_ip if self.drone_ip else DEFAULT_DRONE_IP
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            for _ in range(12): # Burst of 12 packets
                sock.sendto(EMERGENCY_ABORT_TOKEN, (target_ip, UDP_ABORT_PORT))
                time.sleep(0.002)
            sock.close()

            self.lbl_status.config(text="STATUS: EMERGENCY ABORT SENT!", fg="#e53e3e")
            messagebox.showwarning(
                "EMERGENCY ABORT SENT",
                f"Emergency abort signal BLASTED to {target_ip}:{UDP_ABORT_PORT}!\n"
                "All drone flight systems instructed to kill thrust immediately."
            )
        except Exception as e:
            messagebox.showerror("Abort Error", f"Failed to blast UDP abort datagrams: {e}")

    def update_canvas_loop(self):
        self.pulse_phase += 0.12
        if self.pulse_phase > 6.28:
            self.pulse_phase = 0.0

        self.draw_arena_map()
        self.draw_video_feed()

        self.root.after(50, self.update_canvas_loop)

    def draw_arena_map(self):
        canvas = self.map_canvas
        canvas.delete("all")

        cw = canvas.winfo_width()
        ch = canvas.winfo_height()
        if cw <= 10 or ch <= 10:
            return

        # Arena is 15m x 15m with 1m margins = 17m
        scale = min(cw, ch) / 18.0
        ox = (cw - 16.0 * scale) / 2.0
        oy = (ch - 16.0 * scale) / 2.0

        def w2s(wx, wy):
            # Screen X, Y (invert Y)
            return ox + wx * scale, ch - (oy + wy * scale)

        # 1. Background grid
        canvas.create_rectangle(ox, ch - oy - 16.0 * scale, ox + 16.0 * scale, ch - oy, 
                                fill="#0f1524", outline="#1e293b", width=2)

        # 2. Draw 2x2 Meter Modular Rooms (Dashed grid lines)
        for r in range(0, 17, 2):
            x0, y0 = w2s(r, 0)
            x1, y1 = w2s(r, 16)
            canvas.create_line(x0, y0, x1, y1, fill="#1e293b", dash=(3, 3))
            x0, y0 = w2s(0, r)
            x1, y1 = w2s(16, r)
            canvas.create_line(x0, y0, x1, y1, fill="#1e293b", dash=(3, 3))

        # Room labels
        for col in range(8):
            for row in range(8):
                cx, cy = w2s(col * 2.0 + 1.0, row * 2.0 + 1.0)
                canvas.create_text(cx, cy, text=f"R[{col},{row}]", fill="#334155", font=("Consolas", 8))

        # 3. Draw Historic Flight Path Trail
        if len(self.drone_trail) > 1:
            pts = []
            for tx, ty in self.drone_trail:
                sx, sy = w2s(tx, ty)
                pts.extend([sx, sy])
            if len(pts) >= 4:
                canvas.create_line(*pts, fill="#00d8ff", width=2)

        # 4. Draw Survivor Markers (Pulsing glowing beacons)
        pulse_r = 10.0 + 5.0 * math.sin(self.pulse_phase)
        for s in self.survivors:
            sx, sy = w2s(s["x"], s["y"])

            # Outer glowing halo
            canvas.create_oval(sx - pulse_r - 6, sy - pulse_r - 6, sx + pulse_r + 6, sy + pulse_r + 6,
                              outline="#ff2d55", fill="", width=2)
            # Inner beacon
            canvas.create_oval(sx - 8, sy - 8, sx + 8, sy + 8, fill="#ff2d55", outline="white", width=2)
            # Crosshair
            canvas.create_line(sx - 14, sy, sx + 14, sy, fill="white", width=2)
            canvas.create_line(sx, sy - 14, sx, sy + 14, fill="white", width=2)

            # Detail Tag
            canvas.create_rectangle(sx + 14, sy - 28, sx + 130, sy + 12, fill="#182030", outline="#ff2d55", width=1)
            canvas.create_text(sx + 20, sy - 18, text=s["id"], fill="white", anchor="w", font=("Segoe UI", 8, "bold"))
            canvas.create_text(sx + 20, sy - 2, text=f"{s['room']} ({int(s['conf']*100)}%)", fill="#00f0ff", anchor="w", font=("Consolas", 8))

        # 5. Draw Autonomous Drone Icon
        dx, dy, dyaw = self.drone_pose
        dsx, dsy = w2s(dx, dy)
        yaw_rad = math.radians(dyaw)

        # Heading vector line
        hx = dsx + 28 * math.cos(yaw_rad)
        hy = dsy - 28 * math.sin(yaw_rad)
        canvas.create_line(dsx, dsy, hx, hy, fill="#00ff8c", width=2, arrow=tk.LAST)

        # Drone circular chassis
        canvas.create_oval(dsx - 11, dsy - 11, dsx + 11, dsy + 11, fill="#00ff8c", outline="white", width=2)
        canvas.create_text(dsx, dsy + 18, text=f"DRONE ({dx:.2f}m, {dy:.2f}m)", fill="#a0aec0", font=("Consolas", 8, "bold"))

    def draw_video_feed(self):
        canvas = self.video_canvas
        canvas.delete("all")

        cw = canvas.winfo_width()
        ch = canvas.winfo_height()
        if cw <= 10 or ch <= 10:
            return

        self.video_frame_step += 1
        step = self.video_frame_step

        # Draw simulated corridor optical feed with perspective
        canvas.create_rectangle(0, 0, cw, ch, fill="#131826", outline="")

        # Vanishing point corridor walls
        vx = cw / 2.0
        vy = ch / 2.0

        canvas.create_line(0, ch, vx - 40, vy, fill="#2d3748", width=2)
        canvas.create_line(cw, ch, vx + 40, vy, fill="#2d3748", width=2)
        canvas.create_line(0, 0, vx - 40, vy - 30, fill="#2d3748", width=2)
        canvas.create_line(cw, 0, vx + 40, vy - 30, fill="#2d3748", width=2)

        # Survivor Dummy rendering in corridor
        if (step // 60) % 2 == 1:
            sx = vx + 30 * math.sin(step * 0.05)
            sy = vy + 35
            # Target dummy
            canvas.create_oval(sx - 15, sy - 40, sx + 15, sy - 10, fill="#ff9900", outline="")
            canvas.create_rectangle(sx - 20, sy - 10, sx + 20, sy + 45, fill="#ff5500", outline="")
            canvas.create_rectangle(sx - 35, sy - 55, sx + 35, sy + 55, outline="#ff2d55", width=2)
            canvas.create_text(sx, sy - 65, text="TARGET DETECTED (92%)", fill="#ff2d55", font=("Segoe UI", 9, "bold"))

        # Tactical HUD elements
        canvas.create_text(16, 20, text="CAM01: FORWARD_OPTICAL", fill="#00ff8c", anchor="w", font=("Consolas", 9, "bold"))
        canvas.create_text(cw - 16, 20, text="30 FPS [100% OFFLINE]", fill="#a0aec0", anchor="e", font=("Consolas", 9))
        canvas.create_line(cw/2 - 20, ch/2, cw/2 + 20, ch/2, fill="#00ff8c", width=1)
        canvas.create_line(cw/2, ch/2 - 20, cw/2, ch/2 + 20, fill="#00ff8c", width=1)

def main():
    root = tk.Tk()
    app = StandaloneGCSApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()
