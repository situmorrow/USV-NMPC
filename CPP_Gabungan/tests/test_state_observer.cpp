/**
 * @file test_state_observer.cpp
 * @brief Unit Test untuk Modul State Observer & Sinkronisasi Orientasi IMU
 *
 * Menguji:
 * 1. Parsing baris telemetri CSV 8 kolom dari ESP32-S3 Remote-Side
 * 2. Kalibrasi orientasi Yaw IMU dengan penambahan offset +180 derajat
 * 3. Pembentukan vektor state non-dimensional 5x1: [v', r', x', y', psi]
 * 4. Integrasi end-to-end: Telemetry -> Guidance -> State Observer -> NMPC Solver
 */

#include "guidance.hpp"
#include "state_observer.hpp"
#include "telemetry_parser.hpp"
#include "nmpc_kapal_waypoint.h"
#include "nmpc_kapal_waypoint_initialize.h"
#include "nmpc_kapal_waypoint_terminate.h"

#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
  std::cout << "=========================================================================\n";
  std::cout << "  UNIT TEST: STATE OBSERVER & SINKRONISASI ORIENTASI IMU KAPAL           \n";
  std::cout << "=========================================================================\n\n";

  // Parameter Fisik Kapal
  const double L = 1.0107;         // Panjang kapal [m]
  const double u0 = 0.6114;        // Kecepatan surge [m/s]
  const double yaw_offset = 180.0; // Offset kalibrasi yaw kapal

  usv::GuidanceModule guidance(L, u0, 0.10, 3.0);
  usv::StateObserver observer(L, u0, yaw_offset);

  // Inisialisasi Home Point
  const double home_lat = -7.287150;
  const double home_lon = 112.796000;
  guidance.set_home_point(home_lat, home_lon);
  guidance.add_waypoint_geo(-7.286750, 112.796000); // WP 1 (Y ~ 44.5 m)

  // ---------------------------------------------------------------------------
  // TEST 1: Parsing Telemetri Serial & Kalibrasi Yaw (+180 Derajat)
  // ---------------------------------------------------------------------------
  std::cout << "[TEST 1] Memeriksa parsing telemetri dan kalibrasi yaw (+180 deg)...\n";
  
  // Contoh baris telemetri CSV dari ESP32 Remote-Side:
  // timestamp, lat, lon, servo1, servo2, yaw, gyro_z, yaw_rate
  // Raw yaw = 270.0 deg (Dengan offset +180 deg, harus menjadi (270+180)%360 = 90.0 deg menghadap Utara)
  std::string csv_line = "12.450,-7.287150,112.796000,-4.50,-18.20,270.00,0.00,5.20";
  
  auto row_opt = parse_telemetry_line(csv_line);
  assert(row_opt.has_value());
  std::cout << "  -> Raw CSV: " << csv_line << "\n";
  std::cout << "  -> Parsed: t=" << row_opt->timestamp << " s, Lat=" << row_opt->lat 
            << ", Lon=" << row_opt->lon << ", Raw Yaw=" << row_opt->yaw << " deg, YawRate=" << row_opt->yaw_rate << " deg/s\n";

  // Update State Observer
  usv::ShipState state = observer.update(*row_opt, guidance, 0.0);

  std::cout << std::fixed << std::setprecision(2);
  std::cout << "  -> Raw Yaw      : " << state.raw_yaw_deg << " deg\n";
  std::cout << "  -> Offset Yaw   : +" << observer.get_yaw_offset_deg() << " deg\n";
  std::cout << "  -> Calibrated   : " << state.cal_yaw_deg << " deg\n";
  std::cout << "  -> Psi ENU (rad): " << std::setprecision(4) << state.psi_rad << " rad (" 
            << std::setprecision(2) << state.psi_deg << " deg)\n";

  // Verifikasi bahwa (270 + 180) % 360 = 90.0 deg (North)
  assert(std::abs(state.cal_yaw_deg - 90.0) < 1e-3);
  assert(std::abs(state.psi_rad - (M_PI / 2.0)) < 1e-3);
  std::cout << "  [SUCCESS] Kalibrasi offset yaw +180 derajat bekerja dengan tepat!\n\n";

  // ---------------------------------------------------------------------------
  // TEST 2: Validasi Vektor State Non-Dimensional [v', r', x', y', psi]
  // ---------------------------------------------------------------------------
  std::cout << "[TEST 2] Memeriksa pembentukan vektor state non-dimensional 5x1...\n";
  std::cout << "  State Vektor Non-Dimensional untuk NMPC:\n";
  std::cout << "    s[0] (v')   = " << std::setw(8) << state.state_nd[0] << " (sway velocity)\n";
  std::cout << "    s[1] (r')   = " << std::setw(8) << state.state_nd[1] << " (yaw rate non-dim)\n";
  std::cout << "    s[2] (x')   = " << std::setw(8) << state.state_nd[2] << " (posisi East non-dim)\n";
  std::cout << "    s[3] (y')   = " << std::setw(8) << state.state_nd[3] << " (posisi North non-dim)\n";
  std::cout << "    s[4] (psi)  = " << std::setw(8) << state.state_nd[4] << " rad (heading ENU)\n";

  // Nilai ekspektasi:
  // s[0] = 0.0
  // s[1] = (5.20 * pi / 180) * (1.0107 / 0.6114) = 0.090757 * 1.65309 = 0.15003
  // s[2] = 0.0 (berada di Home Point)
  // s[3] = 0.0 (berada di Home Point)
  // s[4] = pi / 2 = 1.570796
  assert(std::abs(state.state_nd[0] - 0.0) < 1e-4);
  assert(std::abs(state.state_nd[1] - 0.1500) < 1e-3);
  assert(std::abs(state.state_nd[2] - 0.0) < 1e-3);
  assert(std::abs(state.state_nd[3] - 0.0) < 1e-3);
  assert(std::abs(state.state_nd[4] - (M_PI / 2.0)) < 1e-4);
  std::cout << "  [SUCCESS] Vektor state non-dimensional 100% konsisten dengan formulasi WyNDA!\n\n";

  // ---------------------------------------------------------------------------
  // TEST 3: Integrasi End-to-End dengan Solver NMPC
  // ---------------------------------------------------------------------------
  std::cout << "[TEST 3] Menguji integrasi Telemetri -> Guidance -> State Observer -> NMPC Solver...\n";
  nmpc_kapal_waypoint_initialize();

  // 1. Dapatkan proyeksi lintasan horizon dari modul guidance
  auto guidance_out = guidance.update_guidance(state.x_enu, state.y_enu);
  assert(guidance_out.has_valid_waypoints);

  // 2. Jalankan solver NMPC
  double u_opt = 0.0;
  double exitflag = 0.0;
  double u_prev = 0.0;

  nmpc_kapal_waypoint(state.state_nd, u_prev, u0,
                      guidance_out.x_ref_seq,
                      guidance_out.y_ref_seq,
                      guidance_out.psi_ref_seq,
                      &u_opt, &exitflag);

  std::cout << "  Hasil Eksekusi NMPC Solver dari State Aktual:\n";
  std::cout << "    -> Exit Flag        : " << exitflag << " (Konvergen / Optimal)\n";
  std::cout << "    -> Rudder Output (u): " << u_opt << " rad (" << (u_opt * 180.0 / M_PI) << " deg)\n";

  assert(exitflag > 0);
  nmpc_kapal_waypoint_terminate();
  std::cout << "  [SUCCESS] Pipeline State Observer -> NMPC berjalan mulus dan konvergen optimal!\n\n";

  std::cout << "=========================================================================\n";
  std::cout << "  SEMUA PENGUJIAN TAHAP 3 (STATE OBSERVER) BERHASIL 100% (PASSED)         \n";
  std::cout << "=========================================================================\n";

  return 0;
}
