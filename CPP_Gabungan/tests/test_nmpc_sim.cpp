/**
 * @file test_nmpc_sim.cpp
 * @brief Standalone Closed-Loop SITL Simulation (Kapal USV + NMPC WyNDA)
 *
 * Mensimulasikan dinamika kapal WyNDA 11-parameter secara tertutup (closed-loop)
 * melintasi 4 titik waypoint tanpa membutuhkan hardware fisik ESP32.
 */

#include "guidance.hpp"
#include "state_observer.hpp"
#include "nmpc_kapal_waypoint.h"
#include "nmpc_kapal_waypoint_initialize.h"
#include "nmpc_kapal_waypoint_terminate.h"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

// 11 Parameter Basis Model Dinamika WyNDA (Sama persis dengan run_nmpc.m)
const double THETA[11] = {
    -9.2816e-01, // theta_1  : v pada v_dot
    -2.6644e-01, // theta_2  : r pada v_dot
     1.2074e-01, // theta_3  : delta pada v_dot
     2.6348e-03, // theta_4  : v pada r_dot
    -1.0577e-02, // theta_5  : r pada r_dot
    -1.3502e-02, // theta_6  : delta pada r_dot
     5.8118e-02, // theta_7  : u0*cos(psi)
     1.4903e-03, // theta_8  : v*sin(psi)
     4.7426e-02, // theta_9  : u0*sin(psi)
    -4.6814e-03, // theta_10 : v*cos(psi)
     4.5806e-02  // theta_11 : r pada psi_dot
};

// Integrasi Euler Model Plant WyNDA (Euler step)
void euler_step_wynda(double s[5], double delta_rad) {
  const double v = s[0];
  const double r = s[1];
  const double psi = s[4];

  const double v_dot = THETA[0] * v + THETA[1] * r + THETA[2] * delta_rad;
  const double r_dot = THETA[3] * v + THETA[4] * r + THETA[5] * delta_rad;
  const double x_dot = THETA[6] * std::cos(psi) - THETA[7] * v * std::sin(psi);
  const double y_dot = THETA[8] * std::sin(psi) + THETA[9] * v * std::cos(psi);
  const double psi_dot = THETA[10] * r;

  s[0] += v_dot;
  s[1] += r_dot;
  s[2] += x_dot;
  s[3] += y_dot;
  s[4] += psi_dot;
  s[4] = usv::wrap_to_pi(s[4]);
}

int main() {
  std::cout << "=========================================================================\n";
  std::cout << "  SIMULASI STANDALONE NMPC WAYPOINT TRACKING USV (MODEL WYNDA C++)       \n";
  std::cout << "=========================================================================\n\n";

  // Parameter Fisik
  const double L = 1.0107;         // Panjang kapal [m]
  const double u0 = 0.6114;        // Kecepatan surge nominal [m/s]
  const double Ts = 0.10;          // Waktu sampling [s]
  const double r_tran = 3.0;       // Radius switching [m]
  const int max_steps = 1500;      // Maksimal 150 detik simulasi

  usv::GuidanceModule guidance(L, u0, Ts, r_tran);

  // Inisialisasi Home Point & 4 Waypoints Uji Kolam
  const double home_lat = -7.287150;
  const double home_lon = 112.796000;
  guidance.set_home_point(home_lat, home_lon);
  guidance.add_waypoint_geo(-7.286750, 112.796000); // WP 1 (0.0, 44.5 m)
  guidance.add_waypoint_geo(-7.286300, 112.796000); // WP 2 (0.0, 94.5 m)
  guidance.add_waypoint_geo(-7.286750, 112.796100); // WP 3 (11.0, 44.5 m)
  guidance.add_waypoint_geo(-7.287150, 112.796000); // WP 4 (0.0, 0.0 m)

  std::cout << "[INIT] Home Point di Lat: " << std::fixed << std::setprecision(6) << home_lat << ", Lon: " << home_lon << "\n";
  std::cout << "[INIT] 4 Titik Waypoint ENU:\n";
  for (size_t i = 0; i < guidance.get_waypoints_enu().size(); ++i) {
    std::cout << "  -> WP #" << (i + 1) << ": East=" << std::setw(6) << std::setprecision(2) << guidance.get_waypoints_enu()[i].x
              << " m, North=" << std::setw(6) << guidance.get_waypoints_enu()[i].y << " m\n";
  }

  // State awal kapal: menghadap Utara (psi = 90 deg = pi/2 rad)
  double state_nd[5] = {0.0, 0.0, 0.0, 0.0, M_PI / 2.0};
  double u_prev_rad = 0.0;

  nmpc_kapal_waypoint_initialize();

  std::cout << "\n[RUN] Memulai loop simulasi mandiri closed-loop...\n";

  int completed_step = max_steps;
  bool reached_goal = false;

  for (int step = 1; step <= max_steps; ++step) {
    const double t = step * Ts;
    const double x_ship = state_nd[2] * L;
    const double y_ship = state_nd[3] * L;
    const double psi_ship_deg = state_nd[4] * 180.0 / M_PI;

    // 1. Update Guidance
    auto g_out = guidance.update_guidance(x_ship, y_ship);

    if (g_out.is_mission_completed) {
      completed_step = step;
      reached_goal = true;
      std::cout << "\n[GOAL] Kapal BERHASIL mencapai Waypoint Terakhir (WP #4)!\n";
      std::cout << "       Waktu tempuh: " << t << " detik (Step " << step << ")\n";
      std::cout << "       Posisi akhir: East = " << x_ship << " m, North = " << y_ship << " m\n";
      break;
    }

    // 2. Eksekusi NMPC
    double u_opt_rad = 0.0;
    double exitflag = 0.0;
    nmpc_kapal_waypoint(state_nd, u_prev_rad, u0,
                        g_out.x_ref_seq, g_out.y_ref_seq, g_out.psi_ref_seq,
                        &u_opt_rad, &exitflag);

    const double rudder_deg = u_opt_rad * 180.0 / M_PI;

    // Cetak progress setiap 50 step (5 detik)
    if (step % 50 == 0 || step == 1) {
      std::cout << std::fixed << std::setprecision(2)
                << "[STEP " << std::setw(4) << step << " | t=" << std::setw(5) << t << "s]"
                << " Pos=(" << std::setw(6) << x_ship << "," << std::setw(6) << y_ship << ")m"
                << " | Psi=" << std::setw(6) << psi_ship_deg << "°"
                << " | Target=WP#" << (g_out.active_wp_index + 1)
                << " (d=" << std::setw(5) << g_out.dist_to_active_wp << "m, Los=" << std::setw(6) << g_out.theta_target_deg << "°)"
                << " | Rudder=" << std::setw(6) << rudder_deg << "°"
                << " | Flag=" << (int)exitflag << "\n";
    }

    // 3. Integrasikan Plant Model WyNDA
    euler_step_wynda(state_nd, u_opt_rad);
    u_prev_rad = u_opt_rad;
  }

  nmpc_kapal_waypoint_terminate();

  std::cout << "\n=========================================================================\n";
  if (reached_goal) {
    std::cout << "  SIMULASI BERHASIL 100%! USV SUKSES MENGIKUTI SELURUH WAYPOINT         \n";
  } else {
    std::cout << "  SIMULASI SELESAI (Batas Step Tercapai)\n";
  }
  std::cout << "=========================================================================\n";

  return 0;
}
