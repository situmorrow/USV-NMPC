#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>   // Untuk mengukur waktu komputasi (high-resolution timer)
#include <iomanip>  // Untuk presisi output pencetakan float/double
#include <fstream>  // Untuk ekspor data simulasi ke file CSV
#include <numeric>  // Untuk perhitungan rata-rata
#include <algorithm>// Untuk mencari min/max

// Include Header Hasil MATLAB Coder
#include "nmpc_kapal.h"
#include "nmpc_kapal_initialize.h"
#include "nmpc_kapal_terminate.h"

// Parameter Utama Kapal & Simulasi
const double L = 101.07;               // Panjang kapal (meter)
const double u_0 = 15.4;               // Kecepatan nominal (m/s)
const double u_0_nd = 1.0;             // Kecepatan non-dimensional
const double T_sim = 1.0;              // Waktu sampling (1 detik per step)
const double dt_nd = T_sim * u_0 / L;   // Step integrator non-dimensional
const int N_STEPS = 100;               // Durasi simulasi 100 detik
const int N_HORIZON = 30;              // Horizon prediksi NMPC
const double PI_VAL = 3.14159265358979323846;

// Model Dinamika Fisik Kapal (Plant System)
struct ShipPlant {
    double A_sys[2][2];
    double B_sys[2];

    void init() {
        double B_ship = 14.0;
        double T_ship = 3.7;
        double m = 2423.0 * 1e3;
        double C_B = 0.65;
        double x_G = 5.25;
        double rho = 1024.0;

        double Y_v_dot = -(1.0 + 0.16 * C_B * B_ship / T_ship - 5.1 * std::pow(B_ship / L, 2)) * PI_VAL * std::pow(T_ship / L, 2);
        double Y_r_dot = -(0.67 * (B_ship / L) - 0.0033 * std::pow(B_ship / T_ship, 2)) * PI_VAL * std::pow(T_ship / L, 2);
        double N_v_dot = -(1.1 * B_ship / L - 0.041 * B_ship / T_ship) * PI_VAL * std::pow(T_ship / L, 2);
        double N_r_dot = -((1.0 / 12.0) + 0.017 * C_B * B_ship / T_ship - 0.33 * B_ship / L) * PI_VAL * std::pow(T_ship / L, 2);
        double Y_v = -(1.0 + 0.4 * C_B * B_ship / T_ship) * PI_VAL * std::pow(T_ship / L, 2);
        double Y_r = -(-0.5 + 2.2 * B_ship / L - 0.08 * B_ship / T_ship) * PI_VAL * std::pow(T_ship / L, 2);
        double N_v = -(0.5 + 2.4 * T_ship / L) * PI_VAL * std::pow(T_ship / L, 2);
        double N_r = -(0.25 + 0.039 * B_ship / T_ship - 0.56 * B_ship / L) * PI_VAL * std::pow(T_ship / L, 2);

        double m_nd = 2.0 * m / (rho * std::pow(L, 3));
        double x_G_nd = x_G / L;
        double I_z_nd = 1.2392e-4;

        double detM = (m_nd - Y_v_dot) * (I_z_nd - N_r_dot) - (m_nd * x_G_nd - Y_r_dot) * (m_nd * x_G_nd - N_v_dot);

        A_sys[0][0] = ((I_z_nd - N_r_dot) * Y_v - (m_nd * x_G_nd - Y_r_dot) * N_v) / detM;
        A_sys[0][1] = ((I_z_nd - N_r_dot) * (Y_r - m_nd * u_0_nd) - (m_nd * x_G_nd - Y_r_dot) * (N_r - m_nd * x_G_nd * u_0_nd)) / detM;
        A_sys[1][0] = ((m_nd - Y_v_dot) * N_v - (m_nd * x_G_nd - N_v_dot) * Y_v) / detM;
        A_sys[1][1] = ((m_nd - Y_v_dot) * (N_r - m_nd * x_G_nd * u_0_nd) - (m_nd * x_G_nd - N_v_dot) * (Y_r - m_nd * u_0_nd)) / detM;

        B_sys[0] = 0.01;
        B_sys[1] = 1.0;
    }

    void dynamics(const double s[5], double u, double s_dot[5]) const {
        double v = s[0];
        double r = s[1];
        double psi = s[4];

        double v_dot = A_sys[0][0] * v + A_sys[0][1] * r + B_sys[0] * u;
        double r_dot = A_sys[1][0] * v + A_sys[1][1] * r + B_sys[1] * u;

        double x_dot = u_0_nd * std::cos(psi) - v * std::sin(psi);
        double y_dot = u_0_nd * std::sin(psi) + v * std::cos(psi);

        s_dot[0] = v_dot;
        s_dot[1] = r_dot;
        s_dot[2] = x_dot;
        s_dot[3] = y_dot;
        s_dot[4] = r;
    }

    void euler_step(double s[5], double u) const {
        double s_dot[5];
        dynamics(s, u, s_dot);
        for (int i = 0; i < 5; i++) {
            s[i] += dt_nd * s_dot[i];
        }
    }
};

int main() {
    // 1. Inisialisasi Solver NMPC (Wajib dipanggil 1 kali di awal)
    nmpc_kapal_initialize();

    // Inisialisasi Plant Kapal
    ShipPlant ship;
    ship.init();

    // 2. Pre-generate Full Trajektori Referensi (Sinusoidal)
    int total_ref_len = N_STEPS + N_HORIZON + 10;
    std::vector<double> x_ref_nd_full(total_ref_len);
    std::vector<double> y_ref_nd_full(total_ref_len);
    std::vector<double> psi_ref_full(total_ref_len);

    for (int i = 0; i < total_ref_len; i++) {
        double t_i = i * T_sim;
        double x_dim = u_0 * t_i;
        double y_dim = 80.0 * std::sin(0.015 * x_dim);
        double dx = u_0;
        double dy = 80.0 * 0.015 * std::cos(0.015 * x_dim) * u_0;
        double psi = std::atan2(dy, dx);

        x_ref_nd_full[i] = x_dim / L;
        y_ref_nd_full[i] = y_dim / L;
        psi_ref_full[i] = psi;
    }

    // 3. Setup State Awal [v; r; x; y; psi] & Rudder Awal
    double current_state_nd[5] = {
        0.0, 
        0.0, 
        x_ref_nd_full[0], 
        y_ref_nd_full[0], 
        psi_ref_full[0]
    };
    double u_prev = 0.0;

    // Vector Penampung Waktu Komputasi
    std::vector<double> comp_times_ms;
    comp_times_ms.reserve(N_STEPS);

    // Buka File CSV untuk Menyimpan Data Simulasi
    std::ofstream csv_file("hasil_simulasi_nmpc_100detik.csv");
    if (csv_file.is_open()) {
        csv_file << "Detik,X_m,Y_m,Psi_deg,v_ms,r_degs,Rudder_deg,Exitflag,Waktu_Komputasi_ms,Waktu_Komputasi_us\n";
    }

    std::cout << "========================================================================================================\n";
    std::cout << "                         SIMULASI NMPC KAPAL 100 DETIK DENGAN UPDATE STATE FISIK                        \n";
    std::cout << "========================================================================================================\n";
    std::cout << std::left 
              << std::setw(8)  << "Detik"
              << std::setw(14) << "Posisi X (m)"
              << std::setw(14) << "Posisi Y (m)"
              << std::setw(16) << "Heading (deg)"
              << std::setw(16) << "Rudder (deg)"
              << std::setw(10) << "Exitflag"
              << std::setw(18) << "Waktu Komputasi"
              << "\n";
    std::cout << "--------------------------------------------------------------------------------------------------------\n";

    // 4. LOOP UTAMA SIMULASI 100 DETIK
    for (int k = 0; k < N_STEPS; k++) {
        // Ekstrak Referensi Horizon N = 30 untuk langkah k
        double x_ref_seq[30];
        double y_ref_seq[30];
        double psi_ref_seq[30];

        for (int i = 0; i < N_HORIZON; i++) {
            x_ref_seq[i]   = x_ref_nd_full[k + i];
            y_ref_seq[i]   = y_ref_nd_full[k + i];
            psi_ref_seq[i] = psi_ref_full[k + i];
        }

        double u_opt = 0.0;
        double exitflag = 0.0;

        // Eksekusi NMPC Solver & Ukur Waktu Komputasi
        auto t_start = std::chrono::high_resolution_clock::now();

        nmpc_kapal(current_state_nd, u_prev, x_ref_seq, y_ref_seq, psi_ref_seq, &u_opt, &exitflag);

        auto t_end = std::chrono::high_resolution_clock::now();

        std::chrono::duration<double, std::milli> elapsed_ms = t_end - t_start;
        std::chrono::duration<double, std::micro> elapsed_us = t_end - t_start;
        comp_times_ms.push_back(elapsed_ms.count());

        // Konversi State & Control ke Satuan Dimensional
        double X_m = current_state_nd[2] * L;
        double Y_m = current_state_nd[3] * L;
        double Psi_deg = current_state_nd[4] * (180.0 / PI_VAL);
        double v_ms = current_state_nd[0] * u_0;
        double r_degs = current_state_nd[1] * (u_0 / L) * (180.0 / PI_VAL);
        double rudder_deg = u_opt * (180.0 / PI_VAL);

        // Cetak Ke Console
        std::cout << std::left 
                  << std::setw(8)  << (k + 1)
                  << std::fixed << std::setprecision(3)
                  << std::setw(14) << X_m
                  << std::setw(14) << Y_m
                  << std::setw(16) << Psi_deg
                  << std::setw(16) << rudder_deg
                  << std::setw(10) << (int)exitflag
                  << std::setprecision(4)
                  << std::setw(18) << (std::to_string(elapsed_ms.count()) + " ms")
                  << "\n";

        // Tulis Data ke CSV
        if (csv_file.is_open()) {
            csv_file << (k + 1) << ","
                     << std::fixed << std::setprecision(4)
                     << X_m << ","
                     << Y_m << ","
                     << Psi_deg << ","
                     << v_ms << ","
                     << r_degs << ","
                     << rudder_deg << ","
                     << exitflag << ","
                     << elapsed_ms.count() << ","
                     << elapsed_us.count() << "\n";
        }

        // UPDATE STATE FISIK KAPAL (Euler Integration 1 Step Detik)
        ship.euler_step(current_state_nd, u_opt);
        u_prev = u_opt;
    }

    if (csv_file.is_open()) {
        csv_file.close();
        std::cout << "--------------------------------------------------------------------------------------------------------\n";
        std::cout << "[INFO] Data posisi dan waktu komputasi berhasil disimpan ke file 'hasil_simulasi_nmpc_100detik.csv'\n";
    }

    // 5. Statistik Komputasi
    double sum_time = std::accumulate(comp_times_ms.begin(), comp_times_ms.end(), 0.0);
    double avg_time = sum_time / N_STEPS;
    double max_time = *std::max_element(comp_times_ms.begin(), comp_times_ms.end());
    double min_time = *std::min_element(comp_times_ms.begin(), comp_times_ms.end());

    std::cout << "========================================================================================================\n";
    std::cout << "                                    STATISTIK WAKTU KOMPUTASI                                           \n";
    std::cout << "========================================================================================================\n";
    std::cout << "Total Waktu Simulasi 100 Detik : " << std::fixed << std::setprecision(4) << sum_time << " ms (" << (sum_time / 1000.0) << " detik)\n";
    std::cout << "Rata-rata Waktu Komputasi/detik: " << std::fixed << std::setprecision(4) << avg_time << " ms\n";
    std::cout << "Waktu Komputasi Tercepat       : " << std::fixed << std::setprecision(4) << min_time << " ms\n";
    std::cout << "Waktu Komputasi Terlambat      : " << std::fixed << std::setprecision(4) << max_time << " ms\n";
    std::cout << "========================================================================================================\n";

    // 6. Terminasi Solver NMPC (Wajib dipanggil saat program ditutup)
    nmpc_kapal_terminate();

    return 0;
}

