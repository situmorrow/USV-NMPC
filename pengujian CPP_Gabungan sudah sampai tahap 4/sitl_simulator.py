#!/usr/bin/env python3
"""
USV SITL (Software-in-the-Loop) Simulator & Virtual ESP32 Bridge
===============================================================
Skrip simulator fisika kapal USV (Model WyNDA 11-Parameter) yang menghubungkan:
1. Python Dashboard ("Local Monitor Dashboard-beta1.5.py") via Virtual COM Port A
2. Program C++ NMPC ("read_write_serial.exe") via Virtual COM Port B atau Subprocess Pipe

Fitur:
- Mensimulasikan dinamika kapal 3-DOF (surge, sway, yaw rate, posisi ENU & GPS) @ 10 Hz
- Mengirim telemetri 24-kolom fixed-point ke Dashboard (tampilan live map, yaw, rudder, status Mini PC)
- Mengirim telemetri 8-kolom CSV ke read_write_serial.exe dan menerima kembali perintah sudut rudder
- Meneruskan perintah $WPSET (Send Way Points) dari Dashboard ke NMPC C++ sebagai stream [WP]
"""

import argparse
import math
import subprocess
import sys
import threading
import time

try:
    import serial
except ImportError:
    serial = None

# Konstanta Fisika Bumi & Model Kapal
R_EARTH = 6371000.0  # Jari-jari bumi (meter)
L_SHIP = 1.0107      # Panjang kapal model (meter)
U0_SURGE = 0.6114    # Kecepatan surge nominal (m/s)
TS = 0.10            # Waktu sampling (10 Hz = 0.10 detik)

# 11 Parameter Basis Model Dinamika Kapal WyNDA (Sama persis dengan run_nmpc.m)
THETA_WYNDA = [
    -9.2816e-01,  # theta_1  : v pada v_dot
    -2.6644e-01,  # theta_2  : r pada v_dot
     1.2074e-01,  # theta_3  : delta pada v_dot
     2.6348e-03,  # theta_4  : v pada r_dot
    -1.0577e-02,  # theta_5  : r pada r_dot
    -1.3502e-02,  # theta_6  : delta pada r_dot
     5.8118e-02,  # theta_7  : u0*cos(psi)
     1.4903e-03,  # theta_8  : v*sin(psi)
     4.7426e-02,  # theta_9  : u0*sin(psi)
    -4.6814e-03,  # theta_10 : v*cos(psi)
     4.5806e-02   # theta_11 : r pada psi_dot
]


def wrap_to_pi(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2.0 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2.0 * math.pi
    return angle_rad


def wrap_to_360(angle_deg: float) -> float:
    deg = angle_deg % 360.0
    if deg < 0:
        deg += 360.0
    return deg


class ShipPlantSimulator:
    """Simulator Dinamika Fisik Kapal USV Model WyNDA"""

    def __init__(self, home_lat=-7.287150, home_lon=112.796000, init_psi_deg=90.0):
        self.home_lat = home_lat
        self.home_lon = home_lon

        # State non-dimensional WyNDA: [v', r', x', y', psi]
        init_psi_rad = math.radians(init_psi_deg)
        self.state_nd = [0.0, 0.0, 0.0, 0.0, init_psi_rad]
        self.rudder_deg = 0.0
        self.rudder_rad = 0.0

        # Status Navigasi
        self.waypoints_enu = []
        self.active_wp_idx = 1
        self.r_tran = 3.0
        self.mission_completed = False

    def set_waypoints(self, wps_geo: list):
        self.waypoints_enu.clear()
        for lat, lon in wps_geo:
            d_lat = math.radians(lat - self.home_lat)
            d_lon = math.radians(lon - self.home_lon)
            lat0_rad = math.radians(self.home_lat)
            y_north = R_EARTH * d_lat
            x_east = R_EARTH * math.cos(lat0_rad) * d_lon
            self.waypoints_enu.append((x_east, y_north))
        self.active_wp_idx = 1
        self.mission_completed = False

    def step(self, rudder_cmd_deg: float):
        self.rudder_deg = max(-35.0, min(35.0, rudder_cmd_deg))
        self.rudder_rad = math.radians(self.rudder_deg)

        # Integrasi Model Diferensial WyNDA (Euler Step)
        v = self.state_nd[0]
        r = self.state_nd[1]
        psi = self.state_nd[4]
        delta = self.rudder_rad

        v_dot = THETA_WYNDA[0] * v + THETA_WYNDA[1] * r + THETA_WYNDA[2] * delta
        r_dot = THETA_WYNDA[3] * v + THETA_WYNDA[4] * r + THETA_WYNDA[5] * delta
        x_dot = THETA_WYNDA[6] * math.cos(psi) - THETA_WYNDA[7] * v * math.sin(psi)
        y_dot = THETA_WYNDA[8] * math.sin(psi) + THETA_WYNDA[9] * v * math.cos(psi)
        psi_dot = THETA_WYNDA[10] * r

        self.state_nd[0] += v_dot
        self.state_nd[1] += r_dot
        self.state_nd[2] += x_dot
        self.state_nd[3] += y_dot
        self.state_nd[4] = wrap_to_pi(self.state_nd[4] + psi_dot)

        # Update Tracking Status
        x_ship, y_ship = self.get_pos_enu()
        if self.waypoints_enu and not self.mission_completed:
            target_x, target_y = self.waypoints_enu[self.active_wp_idx - 1]
            dist = math.hypot(target_x - x_ship, target_y - y_ship)
            if dist <= self.r_tran:
                if self.active_wp_idx < len(self.waypoints_enu):
                    self.active_wp_idx += 1
                else:
                    self.mission_completed = True

    def get_pos_enu(self):
        return (self.state_nd[2] * L_SHIP, self.state_nd[3] * L_SHIP)

    def get_pos_gps(self):
        x_east, y_north = self.get_pos_enu()
        lat0_rad = math.radians(self.home_lat)
        d_lat = y_north / R_EARTH
        d_lon = x_east / (R_EARTH * math.cos(lat0_rad))
        return (self.home_lat + math.degrees(d_lat), self.home_lon + math.degrees(d_lon))

    def get_yaw_deg(self):
        # Heading ENU: 0=East, 90=North
        psi_deg = math.degrees(self.state_nd[4])
        return wrap_to_360(psi_deg)

    def get_raw_sensor_yaw_deg(self):
        # Simulasi IMU fisik dengan offset -180 deg terhadap ENU
        # (sehingga saat C++ menambah +180 deg, hasilnya pas)
        cal_yaw = self.get_yaw_deg()
        raw_yaw = wrap_to_360(cal_yaw - 180.0)
        return raw_yaw

    def get_yaw_rate_dps(self):
        r_rad_s = self.state_nd[1] * (U0_SURGE / L_SHIP)
        return math.degrees(r_rad_s)

    def get_distance_to_active_wp(self):
        if not self.waypoints_enu:
            return 0.0
        x_ship, y_ship = self.get_pos_enu()
        tx, ty = self.waypoints_enu[self.active_wp_idx - 1]
        return math.hypot(tx - x_ship, ty - y_ship)

    def get_heading_setpoint_deg(self):
        if not self.waypoints_enu:
            return self.get_yaw_deg()
        x_ship, y_ship = self.get_pos_enu()
        tx, ty = self.waypoints_enu[self.active_wp_idx - 1]
        los_rad = math.atan2(ty - y_ship, tx - x_ship)
        return wrap_to_360(math.degrees(los_rad))


def run_sitl_bridge(dashboard_port: str, controller_port: str, baud: int = 115200):
    if serial is None:
        print("[ERROR] Pustaka pyserial belum terinstall. Install dengan: pip install pyserial")
        sys.exit(1)

    print("=======================================================================")
    print("  USV SITL VIRTUAL BRIDGE & SHIP SIMULATOR                             ")
    print("=======================================================================")
    print(f"[INFO] Menghubungkan ke Dashboard di Virtual Port : {dashboard_port}")
    print(f"[INFO] Menghubungkan ke NMPC C++ di Virtual Port  : {controller_port}")

    try:
        ser_dash = serial.Serial(dashboard_port, baud, timeout=0.05)
        print(f"[OK] Port Dashboard {dashboard_port} terbuka.")
    except Exception as e:
        print(f"[ERROR] Gagal membuka port Dashboard {dashboard_port}: {e}")
        sys.exit(1)

    try:
        ser_ctrl = serial.Serial(controller_port, baud, timeout=0.05)
        print(f"[OK] Port NMPC Controller {controller_port} terbuka.")
    except Exception as e:
        print(f"[ERROR] Gagal membuka port NMPC Controller {controller_port}: {e}")
        ser_dash.close()
        sys.exit(1)

    # Inisialisasi Simulator Kapal
    plant = ShipPlantSimulator(home_lat=-7.287150, home_lon=112.796000, init_psi_deg=90.0)

    # 4 Waypoints Default Kolam Uji
    default_wps = [
        (-7.286750, 112.796000),
        (-7.286300, 112.796000),
        (-7.286750, 112.796100),
        (-7.287150, 112.796000),
    ]
    plant.set_waypoints(default_wps)

    # Kirim Waypoints awal ke C++ Controller
    print("[INIT] Mengirim default rute waypoint ke NMPC C++...")
    ser_ctrl.write(b"[WP] msg_type=0xA1 home_valid=1 count=4\n")
    ser_ctrl.write(f"[WP] Home: {plant.home_lat:.6f}, {plant.home_lon:.6f}\n".encode("utf-8"))
    for idx, (w_lat, w_lon) in enumerate(default_wps):
        ser_ctrl.write(f"[WP] #{idx+1}: {w_lat:.6f}, {w_lon:.6f}\n".encode("utf-8"))

    latest_rudder_cmd = 0.0
    mini_pc_connected = False
    last_ctrl_rx_time = 0.0

    t_start = time.time()
    step_count = 0

    print("[RUN] Loop Simulasi 10 Hz Aktif. Buka Dashboard dan connect ke COM pasangannya!\n")

    try:
        while True:
            t_now = time.time()
            t_sim = t_now - t_start

            # -----------------------------------------------------------------
            # 1. Baca Data Masuk dari Python Dashboard (User-Side)
            # -----------------------------------------------------------------
            while ser_dash.in_waiting > 0:
                dash_line = ser_dash.readline().decode("utf-8", errors="ignore").strip()
                if not dash_line:
                    continue

                if dash_line.startswith("$WPSET"):
                    print(f"\n[DASHBOARD -> WPSET] Menerima rute waypoint baru dari dashboard:\n  {dash_line}")
                    ser_dash.write(b"$WACK,OK\n")
                    parts = dash_line.split(",")
                    if len(parts) >= 4:
                        h_lat = float(parts[1])
                        h_lon = float(parts[2])
                        wp_count = int(parts[3])
                        plant.home_lat = h_lat
                        plant.home_lon = h_lon
                        wps = []
                        idx = 4
                        for _ in range(wp_count):
                            if idx + 1 < len(parts):
                                wps.append((float(parts[idx]), float(parts[idx + 1])))
                                idx += 2
                        plant.set_waypoints(wps)

                        # Teruskan ke NMPC C++ Controller
                        ser_ctrl.write(f"[WP] msg_type=0xA1 home_valid=1 count={wp_count}\n".encode("utf-8"))
                        ser_ctrl.write(f"[WP] Home: {h_lat:.6f}, {h_lon:.6f}\n".encode("utf-8"))
                        for k, (w_lat, w_lon) in enumerate(wps):
                            ser_ctrl.write(f"[WP] #{k+1}: {w_lat:.6f}, {w_lon:.6f}\n".encode("utf-8"))

                elif dash_line == "$SHUTDOWN":
                    print("[DASHBOARD -> SHUTDOWN] Meneruskan perintah $SHUTDOWN ke Mini PC C++...")
                    ser_dash.write(b"$SACK,OK\n")
                    ser_ctrl.write(b"$SHUTDOWN\n")

            # -----------------------------------------------------------------
            # 2. Baca Respons Kemudi NMPC dari C++ Controller (Remote-Side)
            # -----------------------------------------------------------------
            while ser_ctrl.in_waiting > 0:
                ctrl_line = ser_ctrl.readline().decode("utf-8", errors="ignore").strip()
                if not ctrl_line:
                    continue
                if ctrl_line == "$HB":
                    continue
                if "," in ctrl_line:
                    try:
                        pts = ctrl_line.split(",")
                        rudder_deg = float(pts[1])
                        latest_rudder_cmd = rudder_deg
                        mini_pc_connected = True
                        last_ctrl_rx_time = time.time()
                    except Exception:
                        pass

            if time.time() - last_ctrl_rx_time > 2.0:
                mini_pc_connected = False

            # -----------------------------------------------------------------
            # 3. Kirim Telemetri 8-Kolom ke NMPC C++ Controller @ 10 Hz
            # -----------------------------------------------------------------
            lat_ship, lon_ship = plant.get_pos_gps()
            raw_yaw = plant.get_raw_sensor_yaw_deg()
            yaw_rate = plant.get_yaw_rate_dps()

            # Format 8 kolom: timestamp, lat, lon, servo1, servo2, yaw, gyro_z, yaw_rate
            c_line = (
                f"{t_sim:.3f},{lat_ship:.6f},{lon_ship:.6f},"
                f"{plant.rudder_deg:.2f},{plant.rudder_deg:.2f},"
                f"{raw_yaw:.2f},0.00,{yaw_rate:.2f}\n"
            )
            ser_ctrl.write(c_line.encode("utf-8"))

            # -----------------------------------------------------------------
            # 4. Update Simulasi Fisika Plant Kapal
            # -----------------------------------------------------------------
            plant.step(latest_rudder_cmd)

            # -----------------------------------------------------------------
            # 5. Kirim Telemetri 24-Kolom ke Python Dashboard @ 10 Hz
            # -----------------------------------------------------------------
            speed_mps = U0_SURGE
            yaw_disp = plant.get_yaw_deg()
            heading_sp = plant.get_heading_setpoint_deg()
            heading_err = (heading_sp - yaw_disp + 180.0) % 360.0 - 180.0
            dist_to_wp = plant.get_distance_to_active_wp()
            track_wp_idx = plant.active_wp_index if not plant.mission_completed else 0

            # Format 24 kolom fixed-point (x100 integer strings)
            # 1:timestamp, 2:lat, 3:lon, 4:speed*100, 5:rud1*100, 6:rud2*100, 7:yaw*100,
            # 8:setpoint*100, 9:error*100, 10:rudder_cmd*100, 11:track_wp_index, 12:dist_wp*10,
            # 13-18:accel/gyro, 19-20:rpm, 21-22:battery*100, 23:mode_auto, 24:mini_pc_link
            dash_payload = (
                f"{t_sim:.3f},{lat_ship:.6f},{lon_ship:.6f},"
                f"{int(speed_mps * 100)},"
                f"{int(plant.rudder_deg * 100)},{int(plant.rudder_deg * 100)},"
                f"{int(yaw_disp * 100)},{int(heading_sp * 100)},{int(heading_err * 100)},"
                f"{int(latest_rudder_cmd * 100)},"
                f"{track_wp_idx},{int(dist_to_wp * 10)},"
                f"0,0,100,0,0,{int(yaw_rate * 100)},"
                f"1500,1500,1240,1240,1,{1 if mini_pc_connected else 0}\n"
            )
            ser_dash.write(dash_payload.encode("utf-8"))

            step_count += 1
            if step_count % 50 == 0:
                pos_x, pos_y = plant.get_pos_enu()
                print(
                    f"[SIM t={t_sim:5.1f}s] Pos=({pos_x:6.2f},{pos_y:6.2f})m | "
                    f"Yaw={yaw_disp:5.1f}° | WP#{track_wp_idx} (d={dist_to_wp:5.1f}m) | "
                    f"Rudder={latest_rudder_cmd:5.1f}° | MiniPC={'CONNECTED' if mini_pc_connected else 'WAITING'}"
                )

            time.sleep(TS)

    except KeyboardInterrupt:
        print("\n[INFO] Simulasi dihentikan.")
    finally:
        ser_dash.close()
        ser_ctrl.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="USV SITL Simulator & Virtual Bridge")
    parser.add_argument("--dashboard-port", default="COM2", help="Virtual COM port menuju Dashboard (contoh: COM2)")
    parser.add_argument("--controller-port", default="COM3", help="Virtual COM port menuju C++ NMPC (contoh: COM3)")
    parser.add_argument("--baud", type=int, default=115200, help="Baud rate (default: 115200)")

    args = parser.parse_args()
    run_sitl_bridge(args.dashboard_port, args.controller_port, args.baud)
