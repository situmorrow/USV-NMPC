# USV NMPC C++ Integration (`CPP_Gabungan`)

Repositori integrasi perangkat lunak kendali **Nonlinear Model Predictive Control (NMPC)** berbasis C++ untuk pelacakan *waypoint* (*waypoint tracking*) kapal nirawak (*Unmanned Surface Vehicle* / USV).

Proyek ini menggabungkan:
1. **Komunikasi Serial & Telemetri ESP32-S3:** Dari `Ship_Model_Control_ESP32-S3-v2026.01/Cpp_Files/Cpp_ReadWriteSerial-1.0`
2. **Solver NMPC WyNDA 11 Parameter Basis:** Hasil generate MATLAB Coder C++ dari `USV-NMPC/waypoint/Cpp_2.0/codegen1/`
3. **Modul Guidance & Transformasi Koordinat ENU:** Konversi WGS84 Geodetik (Lat/Lon) $\leftrightarrow$ ENU (Meter), switching radius $r_{\text{tran}} = 3.0\text{ m}$, dan generator horizon $N = 20$.
4. **State Observer & Sinkronisasi Orientasi IMU:** Estimasi state non-dimensional $s = [v', r', x', y', \psi]^T$ dan kalibrasi yaw kapal ($+180^\circ$).
5. **Eksekusi Loop Utama Real-Time (`read_write_serial.exe`):** Siklus kendali 10 Hz otomatis dengan argumen `--rudder-mode nmpc` dan mekanisme *fail-safe*.

---

## 1. Struktur Direktori Proyek

```text
CPP_Gabungan/
├── CMakeLists.txt              # Konfigurasi Build System CMake (GCC/MinGW/MSVC)
├── README.md                   # Dokumentasi teknis & petunjuk penggunaan
├── start_read_write_serial.bat # Skrip startup otomatis untuk Mini PC di kapal
├── startup_guide.md            # Panduan konfigurasi autorun di Windows
├── include/                    # Header file komunikasi, telemetri, guidance & observer
│   ├── guidance.hpp            # Modul Guidance, Proyeksi ENU & Horizon Generator
│   ├── processor.hpp           # Operasi logika & pemrosesan telemetri
│   ├── serial_port.hpp         # Wrapper Serial Port lintas platform (Win32/POSIX)
│   ├── state_observer.hpp      # Estimator State Kapal & Kalibrasi Yaw (+180 deg)
│   └── telemetry_parser.hpp    # Parser CSV telemetri 8 kolom dari ESP32
├── src/                        # Source code aplikasi utama
│   ├── guidance.cpp            # Implementasi konversi ENU, r_tran, & LOS guidance
│   ├── main.cpp                # [TAHAP 4] Loop utama kontroler NMPC & serial bridge
│   ├── serial_port.cpp         # Implementasi driver serial port Windows/Linux
│   └── state_observer.cpp      # Implementasi normalisasi state non-dim WyNDA
├── nmpc/                       # Library C++ NMPC Solver (58 .cpp & 66 .h)
│   ├── nmpc_kapal_waypoint.h   # Entry-point fungsi kendali NMPC
│   ├── nmpc_kapal_waypoint.cpp # Implementasi SQP & model dinamika kapal WyNDA
│   ├── fmincon.cpp             # Non-linear optimization solver SQP
│   └── ... (file pendukung QP, BFGS, LDL factorization)
└── tests/                      # Unit testing & benchmark performa
    ├── test_guidance.cpp       # Unit test guidance, ENU converter & horizon
    ├── test_nmpc.cpp           # Standalone unit test & benchmark kecepatan solver
    └── test_state_observer.cpp # Unit test state observer & kalibrasi yaw (+180 deg)
```

---

## 2. Parameter Kendali, Model Matematika & Opsi CLI

### A. Parameter Model Kapal WyNDA & Navigasi
- **Panjang Kapal ($L$):** $1.0107\text{ m}$
- **Kecepatan Surge Nominal ($u_0$):** $0.6114\text{ m/s}$
- **Waktu Sampling ($T_s$):** $0.10\text{ s}$ ($10\text{ Hz}$)
- **Horizon Prediksi ($N$):** 20 langkah ($2.0\text{ detik}$)
- **Radius Transisi Waypoint ($r_{\text{tran}}$):** $3.0\text{ m}$
- **Offset Kalibrasi Yaw IMU:** $+180.0^\circ$
- **Batas Kemudi Fisik:** $[-35^\circ, +35^\circ]$ ($\Delta u \le 30^\circ/\text{step}$)

### B. Opsi Argumen Baris Perintah (CLI) pada `read_write_serial.exe`
```text
Opsi Komunikasi Serial:
  --port <nama_port>          Port serial (default: COM16 / /dev/ttyUSB0)
  --baud <rate>               Baud rate (default: 115200)
  --timeout <ms>              Timeout baca baris (default: 1000 ms)
  --print <all|csv|wp|nmpc|none> Filter output terminal (default: all)

Opsi Mode Kendali Kemudi (--rudder-mode):
  --rudder-mode <nmpc|zero|yawrate2|demo>
                            nmpc     = Kontrol NMPC Pelacakan Waypoint (Default)
                            zero     = Kemudi netral 0 deg (Safety / Test)
                            yawrate2 = Proporsional yaw_rate * 2 deg (Uji coba)
                            demo     = Operasi matematika field telemetri

Opsi Parameter NMPC & Navigasi:
  --yaw-offset <deg>          Offset kalibrasi yaw kapal (default: 180.0)
  --r-tran <meter>            Radius switching waypoint (default: 3.0 m)
  --surge-speed <m/s>         Kecepatan surge nominal u_0 (default: 0.6114 m/s)
  --ship-length <meter>       Panjang kapal model L (default: 1.0107 m)
  --default-test-wp           Muat 4 titik waypoint default kolam uji
  --home <lat,lon>            Set manual Home Point (contoh: -7.28715,112.79600)
  --add-wp <lat,lon>          Tambah titik waypoint manual (dapat diulang)
```

---

## 3. Kompilasi & Build System

Proyek ini menggunakan **CMake $\ge 3.16$** dengan compiler C++17 (MinGW-w64 GCC atau MSVC) dengan optimasi `-O3` dan `-fopenmp`.

### Langkah Kompilasi (PowerShell / Terminal):

```powershell
# 1. Masuk ke direktori proyek
cd D:\2026\Percobaan_Kapal_Autonomous\USV-NMPC\CPP_Gabungan

# 2. Konfigurasi CMake (MinGW Makefiles)
cmake -B build -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release

# 3. Kompilasi seluruh target
cmake --build build --config Release -j4
```

### Menjalankan Program Utama NMPC:
```powershell
# Jalankan langsung dengan mode NMPC aktif
.\build\read_write_serial.exe --port COM16 --rudder-mode nmpc --print nmpc

# Atau jalankan pengujian dengan waypoint default kolam uji tanpa dashboard
.\build\read_write_serial.exe --port COM16 --rudder-mode nmpc --default-test-wp --print nmpc
```

---

## 4. Alur Loop Siklus 10 Hz & Mekanisme Fail-Safe

Dalam satu siklus kendali ($100\text{ ms}$):
1. **Serial RX:** Membaca baris CSV telemetri 8-kolom dari ESP32 (`timestamp, lat, lon, servo1, servo2, yaw, gyro_z, yaw_rate`).
2. **Tag `[WP]`:** Jika operator mengirim misi waypoint dari dashboard darat, modul guidance langsung menyusun koordinat ENU.
3. **State Observer:** Mengubah posisi GPS kapal saat ini menjadi koordinat ENU $(X_{\text{ship}}, Y_{\text{ship}})$, mengoreksi arah yaw $(+180^\circ)$, dan menyusun vektor state $s = [v', r', x', y', \psi]^T$.
4. **Guidance:** Menghitung jarak ke waypoint aktif, beralih jika $d_{\text{wp}} \le 3.0\text{ m}$, dan menyusun lintasan horizon 20 langkah ke depan ($x_{\text{ref}}, y_{\text{ref}}, \psi_{\text{ref}}$).
5. **NMPC SQP Solver:** Menghitung sudut kemudi optimal $u_{\text{opt}}$.
6. **Mekanisme Fail-Safe:**
   - Jika misi selesai $\rightarrow$ kemudi otomatis netral $0^\circ$.
   - Jika solver gagal konvergen ($\text{exitflag} \le 0$) $\rightarrow$ menggunakan kemudi langkah sebelumnya $u_{\text{prev}}$.
   - Jika Home/Waypoint belum tersedia $\rightarrow$ kemudi netral $0^\circ$.
7. **Serial TX:** Mengirim string `timestamp,rudder_deg` kembali ke ESP32 secara instan.

---

## 5. Rencana Tahapan Selanjutnya (Next Steps)

- [x] **Tahap 1:** Struktur Proyek & Integrasi Library C++ NMPC (**SELESAI**)
- [x] **Tahap 2:** Modul Guidance & Transformasi Koordinat ENU (**SELESAI**)
- [x] **Tahap 3:** Pembentukan State Estimator & Penyelarasan Orientasi Sudut IMU (**SELESAI**)
- [x] **Tahap 4:** Integrasi Penuh Solver NMPC ke dalam Loop Utama `main.cpp` (**SELESAI**)
- [ ] **Tahap 5:** CSV Data Logger Komprehensif untuk Analisis Jurnal/Tesis.
- [ ] **Tahap 6:** Pengujian SITL, HIL, dan Uji Lapangan (*Water Trial*).
