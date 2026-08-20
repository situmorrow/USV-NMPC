/**
 * @file test_guidance.cpp
 * @brief Unit Test untuk Modul Guidance & Transformasi Koordinat ENU
 *
 * Menguji:
 * 1. Parser streaming pesan [WP] dari ESP32-S3
 * 2. Konversi koordinat Geodetik (WGS84 Lat/Lon) -> ENU (East, North meter)
 * 3. Konversi posisi real-time kapal GPS -> ENU
 * 4. Logika switching waypoint r_tran = 3.0 m
 * 5. Generator sekuens horizon referensi NMPC (N = 20)
 */

#include "guidance.hpp"

#include <cassert>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

int main() {
  std::cout << "=========================================================================\n";
  std::cout << "  UNIT TEST: GUIDANCE MODULE & TRANSFORMASI KOORDINAT ENU USV            \n";
  std::cout << "=========================================================================\n\n";

  usv::GuidanceModule guidance(1.0107, 0.6114, 0.10, 3.0);

  // ---------------------------------------------------------------------------
  // TEST 1: Simulasi Parsing Serial Stream [WP] dari ESP32-S3 Remote-Side
  // ---------------------------------------------------------------------------
  std::cout << "[TEST 1] Menguji parsing stream serial [WP] dari ESP32-S3...\n";
  std::vector<std::string> serial_lines = {
      "[WP] msg_type=0xA1 home_valid=1 count=4",
      "[WP] Home: -7.287150, 112.796000",
      "[WP] #1: -7.286750, 112.796000",
      "[WP] #2: -7.286300, 112.796000",
      "[WP] #3: -7.286750, 112.796100",
      "[WP] #4: -7.287150, 112.796000"
  };

  for (const auto& line : serial_lines) {
    bool ok = guidance.parse_waypoint_line(line);
    std::cout << "  -> Parsing: \"" << line << "\" => " << (ok ? "OK" : "IGNORED") << "\n";
  }

  assert(guidance.has_home());
  assert(guidance.get_waypoints_geo().size() == 4);
  assert(guidance.get_waypoints_enu().size() == 4);
  std::cout << "  [SUCCESS] 4 Waypoints dan Home Point berhasil dimuat dari serial.\n\n";

  // ---------------------------------------------------------------------------
  // TEST 2: Validasi Nilai Koordinat ENU dengan Hasil MATLAB run_nmpc.m
  // ---------------------------------------------------------------------------
  std::cout << "[TEST 2] Memeriksa akurasi konversi koordinat Geodetik -> ENU Meter...\n";
  const auto& enu_wps = guidance.get_waypoints_enu();

  std::cout << std::fixed << std::setprecision(2);
  for (size_t i = 0; i < enu_wps.size(); ++i) {
    std::cout << "  WP #" << (i + 1) << " ENU: East (X) = " << std::setw(7) << enu_wps[i].x
              << " m, North (Y) = " << std::setw(7) << enu_wps[i].y << " m\n";
  }

  // Toleransi komparasi dengan MATLAB
  // WP 1: X ~ 0.0 m, Y ~ 44.5 m (dLat = 0.0004 deg -> ~44.48 m)
  assert(std::abs(enu_wps[0].x - 0.00) < 0.1);
  assert(std::abs(enu_wps[0].y - 44.48) < 0.1);

  // WP 2: X ~ 0.0 m, Y ~ 94.5 m (dLat = 0.00085 deg -> ~94.51 m)
  assert(std::abs(enu_wps[1].x - 0.00) < 0.1);
  assert(std::abs(enu_wps[1].y - 94.51) < 0.1);

  // WP 3: X ~ 11.0 m, Y ~ 44.5 m (dLon = 0.0001 deg -> ~11.03 m)
  assert(std::abs(enu_wps[2].x - 11.03) < 0.1);
  assert(std::abs(enu_wps[2].y - 44.48) < 0.1);

  // WP 4: X ~ 0.0 m, Y ~ 0.0 m (kembali ke titik awal)
  assert(std::abs(enu_wps[3].x - 0.00) < 0.1);
  assert(std::abs(enu_wps[3].y - 0.00) < 0.1);
  std::cout << "  [SUCCESS] Seluruh koordinat ENU identik dengan perhitungan MATLAB!\n\n";

  // ---------------------------------------------------------------------------
  // TEST 3: Konversi Posisi Real-Time Kapal (GPS -> ENU)
  // ---------------------------------------------------------------------------
  std::cout << "[TEST 3] Menguji konversi posisi real-time kapal dari data GPS telemetri...\n";
  // Misal kapal berada di dekat WP 1 (Lat: -7.286800, Lon: 112.796020)
  double ship_lat = -7.286800;
  double ship_lon = 112.796020;
  usv::ENUPoint ship_enu = guidance.gps_to_enu(ship_lat, ship_lon);
  std::cout << "  Posisi GPS Kapal : Lat " << std::setprecision(6) << ship_lat << ", Lon " << ship_lon << "\n";
  std::cout << "  Posisi ENU Kapal : X = " << std::setprecision(2) << ship_enu.x << " m, Y = " << ship_enu.y << " m\n";

  // Uji konversi balik ENU -> GPS
  usv::GeoPoint back_gps = guidance.enu_to_gps(ship_enu.x, ship_enu.y);
  if (std::abs(back_gps.lat - ship_lat) >= 1e-7 || std::abs(back_gps.lon - ship_lon) >= 1e-7) {
    std::cerr << "[ERROR] ENU to GPS conversion failed!\n";
    return 1;
  }
  (void)back_gps;
  std::cout << "  [SUCCESS] Konversi dua arah GPS <-> ENU konsisten dan akurat presisi tinggi.\n\n";

  // ---------------------------------------------------------------------------
  // TEST 4: Logika Switching Radius Waypoint (r_tran = 3.0 m)
  // ---------------------------------------------------------------------------
  std::cout << "[TEST 4] Menguji switching radius r_tran (3.0 m)...\n";
  guidance.reset_mission();
  assert(guidance.get_active_wp_index() == 0); // Mulai dari WP #1 (index 0)

  // 1. Kapal di posisi awal (0, 0) -> Target WP 1 (0.0, 44.48)
  auto out1 = guidance.update_guidance(0.0, 0.0);
  std::cout << "  -> Kapal di (0, 0): Active WP index = " << out1.active_wp_index
            << ", Jarak ke WP = " << out1.dist_to_active_wp << " m"
            << ", Target Heading = " << out1.theta_target_deg << " deg\n";
  assert(out1.active_wp_index == 0);
  assert(!out1.is_mission_completed);

  // 2. Kapal mendekati WP 1 pada jarak 2.5 m (masuk radius r_tran <= 3.0 m)
  // Posisi: (0, 42.0) -> Jarak ke WP 1 adalah ~2.48 m
  std::cout << "  -> Kapal mendekati WP 1 di (0, 42.0 m) dalam radius <= 3.0 m...\n";
  auto out2 = guidance.update_guidance(0.0, 42.0);
  std::cout << "  -> Switching Otomatis! Active WP index sekarang = " << out2.active_wp_index
            << " (Target WP #" << (out2.active_wp_index + 1) << ")"
            << ", Jarak ke WP baru = " << out2.dist_to_active_wp << " m"
            << ", Target Heading = " << out2.theta_target_deg << " deg\n";
  assert(out2.active_wp_index == 1); // Otomatis beralih ke WP 2 (index 1)!

  // 3. Kapal mencapai WP 4 (waypoint terakhir) di (0, 0.5 m)
  guidance.update_guidance(0.0, 93.0); // Transisi ke WP 3
  guidance.update_guidance(10.0, 44.0); // Transisi ke WP 4
  auto out_final = guidance.update_guidance(0.0, 1.0); // Masuk r_tran WP 4
  std::cout << "  -> Kapal mencapai WP 4 (Titik Akhir) di (0, 1.0 m):\n";
  std::cout << "     Active WP = " << (out_final.active_wp_index + 1)
            << ", Mission Completed Flag = " << (out_final.is_mission_completed ? "TRUE" : "FALSE") << "\n";
  assert(out_final.is_mission_completed);
  std::cout << "  [SUCCESS] Logika switching r_tran dan deteksi akhir misi bekerja sempurna.\n\n";

  // ---------------------------------------------------------------------------
  // TEST 5: Generator Horizon Referensi NMPC 20-Langkah
  // ---------------------------------------------------------------------------
  std::cout << "[TEST 5] Menguji generator horizon referensi NMPC 20-langkah (N = 20)...\n";
  guidance.reset_mission();
  auto out_horizon = guidance.update_guidance(0.0, 0.0);

  std::cout << "  Sampel 5 langkah horizon pertama (x_ref, y_ref non-dimensional, psi_ref rad):\n";
  for (int h = 0; h < 5; ++h) {
    std::cout << "    Step " << (h + 1) << ": x_ref = " << std::setw(6) << out_horizon.x_ref_seq[h]
              << ", y_ref = " << std::setw(6) << out_horizon.y_ref_seq[h]
              << ", psi_ref = " << out_horizon.psi_ref_seq[h] << " rad ("
              << (out_horizon.psi_ref_seq[h] * 180.0 / M_PI) << " deg)\n";
  }

  // Verifikasi sifat horizon referensi
  // Kecepatan u0 = 0.6114 m/s, Ts = 0.1 s, L = 1.0107 m
  // Pada step 1: dist = 0.06114 m -> y_ref = 0.06114 / 1.0107 = 0.06049
  assert(std::abs(out_horizon.x_ref_seq[0] - 0.0) < 1e-3);
  assert(std::abs(out_horizon.y_ref_seq[0] - 0.06049) < 1e-3);
  assert(std::abs(out_horizon.psi_ref_seq[0] - (M_PI / 2.0)) < 1e-3);
  std::cout << "  [SUCCESS] Horizon referensi NMPC 20-langkah tepat dan siap diinputkan ke solver NMPC!\n\n";

  std::cout << "=========================================================================\n";
  std::cout << "  SEMUA PENGUJIAN TAHAP 2 (GUIDANCE & ENU) BERHASIL 100% (PASSED)         \n";
  std::cout << "=========================================================================\n";

  return 0;
}
