/**
 * @file test_nmpc.cpp
 * @brief Standalone Unit Test & Benchmark untuk NMPC Solver C++
 *
 * Menguji fungsionalitas dan kecepatan komputasi solver NMPC hasil generate MATLAB Coder
 * pada model kapal WyNDA (11 Parameter Basis).
 */

#include "nmpc_kapal_waypoint.h"
#include "nmpc_kapal_waypoint_initialize.h"
#include "nmpc_kapal_waypoint_terminate.h"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

int main() {
  std::cout << "=========================================================================\n";
  std::cout << "  UNIT TEST & BENCHMARK NMPC SOLVER (KAPAL WYNDA 11 BASIS PARAMETER)      \n";
  std::cout << "=========================================================================\n\n";

  // Inisialisasi library NMPC
  nmpc_kapal_waypoint_initialize();
  std::cout << "[INIT] NMPC library initialized successfully.\n";

  // Parameter Fisik Kapal (Sesuai run_nmpc.m)
  const double L = 1.0107;         // Panjang kapal [m]
  const double u_0 = 0.6114;       // Kecepatan surge nominal [m/s]
  const double T_sim = 0.10;       // Periode sampling 10 Hz [s]
  const int N = 20;                // Horizon prediksi

  // Kondisi Awal Kapal: Posisi (0,0), Heading 90 deg (Menghadap Utara)
  const double heading_init_rad = 90.0 * (M_PI / 180.0);
  double current_state_nd[5] = {0.0, 0.0, 0.0, 0.0, heading_init_rad}; // [v', r', x', y', psi]
  double u_prev = 0.0; // Sudut kemudi sebelumnya (radian)

  // Target Waypoint: (X: 10 m, Y: 44.5 m)
  const double wp_target_x = 10.0;
  const double wp_target_y = 44.5;
  const double theta_target = std::atan2(wp_target_y - 0.0, wp_target_x - 0.0);

  // Buat Sekuens Horizon Referensi N-Langkah (Non-dimensional untuk X dan Y)
  double x_ref_seq[20];
  double y_ref_seq[20];
  double psi_ref_seq[20];

  for (int h = 0; h < N; ++h) {
    const double step_idx = static_cast<double>(h + 1);
    const double dist_step = step_idx * u_0 * T_sim;
    const double x_ref_dim = 0.0 + dist_step * std::cos(theta_target);
    const double y_ref_dim = 0.0 + dist_step * std::sin(theta_target);

    x_ref_seq[h] = x_ref_dim / L;
    y_ref_seq[h] = y_ref_dim / L;
    psi_ref_seq[h] = theta_target;
  }

  std::cout << "[TEST] Target Heading (theta_target) : " << (theta_target * 180.0 / M_PI) << " deg\n";
  std::cout << "[TEST] Current Heading (psi)        : " << (heading_init_rad * 180.0 / M_PI) << " deg\n";
  std::cout << "[TEST] Horizon Length (N)           : " << N << " steps (" << (N * T_sim) << " s)\n\n";

  // Eksekusi Warm-up 1 Iterasi
  double u_opt = 0.0;
  double exitflag = 0.0;

  nmpc_kapal_waypoint(current_state_nd, u_prev, u_0, x_ref_seq, y_ref_seq, psi_ref_seq, &u_opt, &exitflag);

  std::cout << "[RESULT WARM-UP]\n";
  std::cout << "  -> Exit Flag : " << exitflag << " (" << (exitflag > 0 ? "SUCCESS / CONVERGED" : "FAILED") << ")\n";
  std::cout << "  -> Optimal Rudder u_opt : " << u_opt << " rad (" << (u_opt * 180.0 / M_PI) << " deg)\n\n";

  // Benchmark Kecepatan Solver (50 Iterasi berturut-turut)
  const int BENCHMARK_RUNS = 50;
  std::cout << "[BENCHMARK] Menjalankan " << BENCHMARK_RUNS << " iterasi untuk mengukur waktu komputasi...\n";

  std::vector<double> elapsed_times_ms;
  elapsed_times_ms.reserve(BENCHMARK_RUNS);

  for (int iter = 0; iter < BENCHMARK_RUNS; ++iter) {
    const auto t_start = std::chrono::high_resolution_clock::now();

    nmpc_kapal_waypoint(current_state_nd, u_prev, u_0, x_ref_seq, y_ref_seq, psi_ref_seq, &u_opt, &exitflag);

    const auto t_end = std::chrono::high_resolution_clock::now();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
    elapsed_times_ms.push_back(elapsed_ms);

    // Update u_prev untuk langkah berikutnya
    u_prev = u_opt;
  }

  double total_time_ms = 0.0;
  double min_time_ms = elapsed_times_ms[0];
  double max_time_ms = elapsed_times_ms[0];

  for (double t : elapsed_times_ms) {
    total_time_ms += t;
    if (t < min_time_ms) min_time_ms = t;
    if (t > max_time_ms) max_time_ms = t;
  }

  const double avg_time_ms = total_time_ms / BENCHMARK_RUNS;

  std::cout << std::fixed << std::setprecision(2);
  std::cout << "-------------------------------------------------------------------------\n";
  std::cout << "  HASIL BENCHMARK WAKTU KOMPUTASI NMPC C++:\n";
  std::cout << "  - Rata-rata per step : " << avg_time_ms << " ms\n";
  std::cout << "  - Minimum per step   : " << min_time_ms << " ms\n";
  std::cout << "  - Maksimum per step  : " << max_time_ms << " ms\n";
  std::cout << "  - Target Real-Time   : < 100.0 ms (Loop 10 Hz)\n";
  if (avg_time_ms < 100.0) {
    std::cout << "  [STATUS] REAL-TIME CAPABLE (Margin Keamanan: " 
              << ((100.0 - avg_time_ms) / 100.0 * 100.0) << "%)\n";
  } else {
    std::cout << "  [STATUS] PERINGATAN: Melebihi batas waktu sampling 100 ms!\n";
  }
  std::cout << "-------------------------------------------------------------------------\n\n";

  // Terminasi library NMPC
  nmpc_kapal_waypoint_terminate();
  std::cout << "[DONE] NMPC library terminated cleanly.\n";

  return (exitflag > 0) ? 0 : 1;
}
