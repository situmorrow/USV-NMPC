#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>   // Untuk mengukur waktu komputasi (high-resolution timer)
#include <iomanip>  // Untuk presisi output pencetakan float/double

// Include Header Hasil MATLAB Coder
#include "nmpc_kapal.h"
#include "nmpc_kapal_initialize.h"
#include "nmpc_kapal_terminate.h"

int main() {
    // 1. Inisialisasi Solver NMPC (Wajib dipanggil 1 kali di awal)
    nmpc_kapal_initialize();

    // 2. Data State & Referensi Horizon
    double current_state_nd[5] = {0.0, 0.0, 0.0, 0.98, 0.0}; // [v; r; x; y; psi]
    double u_prev = 0.0;                                     // Kemudi awal (0 rad)

    // Sequential Horizon N = 30
    double x_ref_seq[30];
    double y_ref_seq[30];
    double psi_ref_seq[30];

    // Isi data referensi horizon N=30 (contoh sederhana)
    for (int i = 0; i < 30; i++) {
        x_ref_seq[i] = (i + 1) * 0.1;
        y_ref_seq[i] = 0.98;
        psi_ref_seq[i] = 0.0;
    }

    // Variabel Penampung Output
    double u_opt = 0.0;
    double exitflag = 0.0;

    // 3. PEMANGGILAN ENTRY POINT FUNCTION 1-STEP NMPC & MENGUKUR WAKTU KOMPUTASI
    auto t_start = std::chrono::high_resolution_clock::now();

    nmpc_kapal(current_state_nd, u_prev, x_ref_seq, y_ref_seq, psi_ref_seq, &u_opt, &exitflag);

    auto t_end = std::chrono::high_resolution_clock::now();

    // Menghitung durasi dalam milidetik (ms) dan mikrodetik (us)
    std::chrono::duration<double, std::milli> elapsed_ms = t_end - t_start;
    std::chrono::duration<double, std::micro> elapsed_us = t_end - t_start;

    // Konversi hasil u_opt (radian) ke derajat untuk dikirim ke ESP32
    double rudder_deg = u_opt * (180.0 / 3.14159265358979323846);

    std::cout << "=========================================================" << std::endl;
    std::cout << "         HASIL EKSEKUSI SOLVER NMPC KAPAL (C++)          " << std::endl;
    std::cout << "=========================================================" << std::endl;
    std::cout << "Status Solver Exitflag : " << exitflag << std::endl;
    std::cout << "Sinyal Rudder Optimal  : " << std::fixed << std::setprecision(4) 
              << rudder_deg << " derajat" << std::endl;
    std::cout << "Waktu Komputasi        : " << std::fixed << std::setprecision(4) 
              << elapsed_ms.count() << " ms (" 
              << elapsed_us.count() << " us)" << std::endl;
    std::cout << "=========================================================" << std::endl;

    // 4. Terminasi Solver NMPC (Wajib dipanggil saat program ditutup)
    nmpc_kapal_terminate();

    return 0;
}
