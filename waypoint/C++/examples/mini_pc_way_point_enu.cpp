#include <iostream>     // Untuk standar input/output stream (std::cout, std::cerr)
#include <vector>       // Untuk tipe data kontainer dinamis std::vector
#include <string>       // Untuk manipulasi teks std::string
#include <sstream>      // Untuk operasi stringstream (parsing/split string)
#include <cmath>        // Untuk fungsi matematika dasar
#include <chrono>       // Untuk pengukuran waktu eksekusi high-resolution (millisecond precision)
#include <thread>       // Untuk fungsi penundaan waktu (std::this_thread::sleep_for)
#include <windows.h>    // API Windows untuk komunikasi Serial Port (Win32 Serial API)

// Header C++ hasil ekspor/codegen dari MATLAB Coder untuk NMPC Kapal Waypoint
#include "nmpc_kapal_waypoint.h"
#include "nmpc_kapal_waypoint_initialize.h"
#include "nmpc_kapal_waypoint_terminate.h"

// Definisi konstanta Pi untuk konversi derajat <-> radian
constexpr double M_PI_VAL = 3.14159265358979323846;

// ============================================================================
// KONSTANTA FISIK & KONFIGURASI NAVIGASI (Paper Potočnik 2025)
// ============================================================================
const double R_EARTH = 6371000.0;     // Jari-jari bumi (meter)
const double R_TRAN = 5.0;             // Switching Radius r_tran (meter)
const double SHIP_LENGTH_L = 1.0;      // Panjang kapal L (meter) untuk Scaling ND
const double U_NOMINAL = 1.0;          // Kecepatan nominal u_0 (m/s)
const int HORIZON_N = 30;              // Horizon Prediksi N-Step NMPC (sesuai codegen signature)
const double DT_NMPC = 0.1;            // Sampling time dt (0.1 detik / 10 Hz)

// Struct Data Koordinat
struct WaypointGeo { double lat; double lon; };
struct WaypointENU { double x;   double y;   };

// Variabel Global Orchestrator Misi
bool g_is_origin_locked = false;
double g_lat_0 = 0.0;
double g_lon_0 = 0.0;
size_t g_active_wp_index = 0;

// Daftar Waypoint Misi (Latitude, Longitude)
std::vector<WaypointGeo> g_waypoints_geo = {
    {-7.275812, 112.793145}, // WP 0 (Start)
    {-7.275900, 112.793500}, // WP 1
    {-7.276100, 112.793800}  // WP 2 (Goal)
};
std::vector<WaypointENU> g_waypoints_enu;

// ============================================================================
// FUNGSI KONVERSI LAT/LON KE ENU (FLAT EARTH) & INISIALISASI MISI
// ============================================================================
/**
 * @brief Mengonversi koordinat geografis (Lat, Lon) ke koordinat Kartesian ENU (East, North) dalam meter.
 */
WaypointENU convertLatLonToENU(double lat, double lon, double lat_ref, double lon_ref) {
    double dLat = (lat - lat_ref) * (M_PI_VAL / 180.0);
    double dLon = (lon - lon_ref) * (M_PI_VAL / 180.0);
    
    WaypointENU res;
    res.y = R_EARTH * dLat;                                   // North (meter)
    res.x = R_EARTH * std::cos(lat_ref * (M_PI_VAL / 180.0)) * dLon; // East (meter)
    return res;
}

/**
 * @brief Mengunci Home Point (Origin) dan mengonversi seluruh waypoint geografis ke ENU Meter.
 */
void initializeMission(double start_lat, double start_lon) {
    g_lat_0 = start_lat;
    g_lon_0 = start_lon;
    g_is_origin_locked = true;

    g_waypoints_enu.clear();
    for (const auto& wp : g_waypoints_geo) {
        WaypointENU enu = convertLatLonToENU(wp.lat, wp.lon, g_lat_0, g_lon_0);
        g_waypoints_enu.push_back(enu);
    }
    std::cout << "[INIT] Origin Locked at (" << g_lat_0 << ", " << g_lon_0 << "). "
              << g_waypoints_enu.size() << " Waypoints converted to ENU Meter.\n";
}

/**
 * @brief Kelas SerialPort menggunakan Win32 API.
 * Digunakan untuk komunikasi serial USB dua arah antara Mini PC (C++ NMPC Solver) 
 * dan ESP32-S3 (Low-Level Controller / Telemetry Unit).
 */
class SerialPort {
private:
    HANDLE hSerial;    // Handle file serial port Windows
    bool connected;    // Status apakah port serial berhasil dibuka dan dikonfigurasi

public:
    /**
     * @brief Konstruktor untuk membuka dan mengonfigurasi serial port Win32.
     * @param portName Nama port (misal "COM3", "COM4")
     * @param baudRate Kecepatan transmisi (default 115200 bps)
     * @param quiet Mode diam (true = tidak mencetak error jika port gagal dibuka, cocok untuk auto-scan)
     */
    SerialPort(const char* portName, DWORD baudRate = CBR_115200, bool quiet = false) {
        // Pada Windows, port COM di atas COM9 atau format umum aman menggunakan prefix "\\.\COMx"
        std::string fullPortName = "\\\\.\\" + std::string(portName);
        
        // Membuka perangkat serial sebagai file sistem (Read/Write mode)
        hSerial = CreateFileA(
            fullPortName.c_str(), 
            GENERIC_READ | GENERIC_WRITE, 
            0,                      // Tidak ada berbagi akses (exclusive access)
            NULL,                   // Atribut keamanan default
            OPEN_EXISTING,          // Membuka perangkat yang sudah ada
            FILE_ATTRIBUTE_NORMAL,  // Atribut berkas standar
            NULL
        );
        
        // Cek jika gagal membuka port
        if (hSerial == INVALID_HANDLE_VALUE) {
            connected = false;
            if (!quiet) {
                std::cerr << "[ERROR] Gagal membuka port serial " << portName << "! Cek koneksi USB atau COM port." << std::endl;
            }
            return;
        }

        // Membaca pengaturan Device Control Block (DCB) saat ini
        DCB dcbParams = { 0 };
        dcbParams.DCBlength = sizeof(dcbParams);

        if (!GetCommState(hSerial, &dcbParams)) {
            connected = false;
            CloseHandle(hSerial);
            return;
        }

        // Konfigurasi Parameter Serial: BaudRate, Data Bits 8, Stop Bit 1, No Parity (8N1)
        dcbParams.BaudRate = baudRate;
        dcbParams.ByteSize = 8;
        dcbParams.StopBits = ONESTOPBIT;
        dcbParams.Parity   = NOPARITY;
        dcbParams.fDtrControl = DTR_CONTROL_ENABLE; // Mengaktifkan DTR agar ESP32 tidak menggantung

        // Menerapkan konfigurasi ke serial port
        if (!SetCommState(hSerial, &dcbParams)) {
            connected = false;
            CloseHandle(hSerial);
            return;
        }

        // Pengaturan Timeout Baca/Tulis Serial (mencegah I/O blocking selamanya)
        COMMTIMEOUTS timeouts = { 0 };
        timeouts.ReadIntervalTimeout         = 5;    // Waktu jeda antarkarakter (ms)
        timeouts.ReadTotalTimeoutConstant     = 10;  // Waktu tunggu total pembacaan (ms)
        timeouts.ReadTotalTimeoutMultiplier   = 1;
        timeouts.WriteTotalTimeoutConstant    = 10;  // Waktu tunggu total penulisan (ms)
        timeouts.WriteTotalTimeoutMultiplier  = 1;

        SetCommTimeouts(hSerial, &timeouts);
        connected = true;
    }

    /**
     * @brief Destruktor untuk menutup handle serial port saat objek dihancurkan.
     */
    ~SerialPort() {
        if (connected) {
            CloseHandle(hSerial);
        }
    }

    /**
     * @brief Memeriksa status koneksi serial.
     * @return true jika serial port terhubung dan siap.
     */
    bool isConnected() const { return connected; }

    /**
     * @brief Membaca satu baris string dari serial buffer sampai karakter '\n'.
     * @param line Variable string output penampung baris telemetry.
     * @return true jika berhasil membaca setidaknya satu karakter non-empty.
     */
    bool readLine(std::string &line) {
        line.clear();
        char ch;
        DWORD bytesRead;
        // Membaca byte demi byte hingga menemukan baris baru (\n)
        while (ReadFile(hSerial, &ch, 1, &bytesRead, NULL) && bytesRead > 0) {
            if (ch == '\n') return !line.empty(); // Selesai membaca 1 baris
            if (ch != '\r') line += ch;           // Abaikan Carriage Return '\r'
        }
        return !line.empty();
    }

    /**
     * @brief Membaca seluruh data buffer serial yang tersedia secara non-blocking.
     * @return std::string data teks buffer serial yang terbaca.
     */
    std::string readString() {
        std::string result;
        char buffer[512];
        DWORD bytesRead = 0;
        if (ReadFile(hSerial, buffer, sizeof(buffer) - 1, &bytesRead, NULL) && bytesRead > 0) {
            buffer[bytesRead] = '\0';
            result = std::string(buffer);
        }
        return result;
    }

    /**
     * @brief Mengirimkan data string ke serial port (ESP32).
     * @param data String perintah yang akan dikirim.
     * @return true jika berhasil terkirim.
     */
    bool writeString(const std::string &data) {
        DWORD bytesWritten;
        return WriteFile(hSerial, data.c_str(), (DWORD)data.length(), &bytesWritten, NULL);
    }
};

/**
 * @brief Fungsi pembantu untuk memecah (parsing) string berdasarkan karakter pemisah (delimiter).
 * @param s String masukan yang akan dipecah (misal telemetry "$SHIP,lat,lon,...")
 * @param delim Karakter pemisah (misal koma ',')
 * @return std::vector<std::string> kumpulan token string hasil pecahan
 */
std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

/**
 * @brief Otomatis memindai port virtual COM (COM1 - COM20) untuk menemukan ESP32
 * yang memancarkan data telemetri $SHIP.
 * @return std::string Nama port yang terdeteksi (misal "COM4"), atau string kosong jika belum terdeteksi.
 */
std::string autoDetectESP32Port() {
    std::cout << "[AUTO-DETECT] Memindai Port Virtual COM (COM1 - COM20)...\n";
    for (int i = 1; i <= 20; ++i) {
        std::string portName = "COM" + std::to_string(i);
        SerialPort testSerial(portName.c_str(), CBR_115200, true /* quiet mode */);
        
        if (testSerial.isConnected()) {
            // Tunggu 200ms untuk menampung telemetri $SHIP dari ESP32 ke buffer serial
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            std::string sampleData = testSerial.readString();
            
            if (sampleData.find("$SHIP,") != std::string::npos) {
                std::cout << "[SUCCESS] ESP32 Terdeteksi pada Port: " << portName << "\n";
                return portName;
            }
        }
    }
    return ""; // Belum terdeteksi
}

/**
 * @brief Memproses 1 langkah iterasi kalkulasi NMPC berdasarkan baris telemetri $SHIP (Paper Potočnik 2025).
 * @param serial Objek SerialPort aktif
 * @param line String telemetri dari ESP32
 * @param u_prev Referensi ke nilai kontrol sebelumnya u[k-1] (radian)
 * @param step_counter Referensi ke penghitung langkah iterasi
 */
void processNMPCStep(SerialPort &serial, const std::string &line, double &u_prev, int &step_counter) {
    // Memeriksa apakah pesan serial diawali dengan header standar telemetry "$SHIP,"
    if (line.rfind("$SHIP,", 0) == 0) {
        std::string cleanLine = line;
        // Menghapus karakter asteris '*' di akhir pesan jika ada
        if (!cleanLine.empty() && cleanLine.back() == '*') cleanLine.pop_back();

        // Memecah string telemetry berdasarkan koma ','
        // Format telemetry ESP32: $SHIP,lat,lon,yaw_deg,r_dps,v_mps,act_rdr
        std::vector<std::string> tokens = split(cleanLine, ',');
        if (tokens.size() >= 7) {
            try {
                // 1. Ekstraksi Data Telemetry Sensor Real-Time
                double lat     = std::stod(tokens[1]); // Latitude GPS
                double lon     = std::stod(tokens[2]); // Longitude GPS
                double yaw_deg = std::stod(tokens[3]); // Sudut Yaw Heading (Derajat)
                double r_dps   = std::stod(tokens[4]); // Laju Yaw Rate (Derajat/detik)
                double v_mps   = std::stod(tokens[5]); // Kecepatan Kapal (m/s)
                double act_rdr = std::stod(tokens[6]); // Sudut Kemudi Riil Aktuator (Derajat)

                step_counter++;

                // LANGKAH A: Kunci Origin pada Paket GPS Valid Pertama
                if (!g_is_origin_locked) {
                    initializeMission(lat, lon);
                }

                // LANGKAH B: Konversi Posisi Kapal Real-Time dari Lat/Lon ke ENU Meter
                WaypointENU ship_enu = convertLatLonToENU(lat, lon, g_lat_0, g_lon_0);

                // Cek apakah Misi Sudah Selesai (Sampai Waypoint Akhir)
                if (g_active_wp_index >= g_waypoints_enu.size()) {
                    std::cout << "[MISSION COMPLETE] Kapal telah tiba di Waypoint Tujuan Akhir!\n";
                    std::string stop_cmd = "$NMPC,0.0,0.0*\n";
                    serial.writeString(stop_cmd);
                    return;
                }

                WaypointENU active_wp = g_waypoints_enu[g_active_wp_index];

                // LANGKAH C: Evaluasi Switching Radius Logic (r_tran) ala Paper Potočnik
                double dist_to_wp = std::hypot(active_wp.x - ship_enu.x, active_wp.y - ship_enu.y);

                if (dist_to_wp <= R_TRAN) {
                    std::cout << "[WAYPOINT SWITCH] Masuk zona radius r_tran (" << R_TRAN << " m) dari WP " 
                              << g_active_wp_index << ". Ganti target ke WP " << (g_active_wp_index + 1) << "\n";
                    
                    g_active_wp_index++;
                    
                    if (g_active_wp_index >= g_waypoints_enu.size()) {
                        std::cout << "[MISSION COMPLETE] Kapal telah tiba di Waypoint Terakhir!\n";
                        std::string stop_cmd = "$NMPC,0.0,0.0*\n";
                        serial.writeString(stop_cmd);
                        return;
                    }
                    active_wp = g_waypoints_enu[g_active_wp_index];
                    dist_to_wp = std::hypot(active_wp.x - ship_enu.x, active_wp.y - ship_enu.y);
                }

                // LANGKAH D: Hitung Target Heading Angle (theta_Target) dalam Koordinat ENU
                double theta_target = std::atan2(active_wp.y - ship_enu.y, active_wp.x - ship_enu.x);

                // LANGKAH E: Generate Dynamic N-Step Horizon Reference (30 Steps) untuk NMPC
                double x_ref_seq[30];
                double y_ref_seq[30];
                double psi_ref_seq[30];

                for (int h = 0; h < HORIZON_N; ++h) {
                    double dist_step = (h + 1) * U_NOMINAL * DT_NMPC;
                    
                    // Garis acuan lurus dari posisi kapal ke arah theta_target (dalam satuan ND)
                    x_ref_seq[h]   = (ship_enu.x + dist_step * std::cos(theta_target)) / SHIP_LENGTH_L;
                    y_ref_seq[h]   = (ship_enu.y + dist_step * std::sin(theta_target)) / SHIP_LENGTH_L;
                    psi_ref_seq[h] = theta_target;
                }

                // LANGKAH F: Konversi State Kapal ke Bentuk Tanpa Dimensi (Non-Dimensional State Conversion)
                double psi_rad = yaw_deg * (M_PI_VAL / 180.0);
                double current_state_nd[5] = {
                    v_mps / U_NOMINAL,
                    (r_dps * M_PI_VAL / 180.0) * (SHIP_LENGTH_L / U_NOMINAL),
                    ship_enu.x / SHIP_LENGTH_L,
                    ship_enu.y / SHIP_LENGTH_L,
                    psi_rad
                };

                double u_opt = 0.0;    // Variabel penampung hasil komputasi kemudi optimal (radian)
                double exitflag = 0.0; // Status konvergensi solver NMPC

                // LANGKAH G: Eksekusi NMPC Solver (Optimization 1-step Receding Horizon)
                auto t_start = std::chrono::high_resolution_clock::now();
                
                // Pemanggilan fungsi C++ NMPC utama dari MATLAB Coder
                nmpc_kapal_waypoint(current_state_nd, u_prev, x_ref_seq, y_ref_seq, psi_ref_seq, &u_opt, &exitflag);
                
                auto t_end = std::chrono::high_resolution_clock::now();

                // Menghitung durasi komputasi solver dalam milidetik (ms)
                std::chrono::duration<double, std::milli> elapsed_ms = t_end - t_start;

                // Update aksi kontrol sebelumnya untuk iterasi berikutnya
                u_prev = u_opt;

                // Konversi perintah kemudi dari radian ke derajat untuk aktuator
                double rudder_deg = u_opt * (180.0 / M_PI_VAL);

                // LANGKAH H: Pembentukan & Pengiriman Perintah Kontrol Serial ke ESP32
                // Format perintah: $NMPC,rudder_angle_deg,throttle_percentage*
                std::string cmd = "$NMPC," + std::to_string(rudder_deg) + ",50.0*\n";
                serial.writeString(cmd);

                // LANGKAH I: Logging/Monitoring Real-Time pada Konsol Mini PC
                std::cout << "[Step " << step_counter << "] RX: Lat=" << lat << " Lon=" << lon 
                          << " Yaw=" << yaw_deg << "° | Active WP: " << g_active_wp_index 
                          << " | Dist: " << dist_to_wp << " m"
                          << " | Target Heading: " << (theta_target * 180.0 / M_PI_VAL) << "°"
                          << " | Rudder Cmd: " << rudder_deg << "°"
                          << " (Time: " << elapsed_ms.count() << " ms) -> TX: " << cmd;

            } catch (const std::exception &e) {
                // Menangani error jika ada kesalahan format atau tipe data pada string telemetry
                std::cerr << "[WARN] Parsing Error: " << e.what() << std::endl;
            }
        }
    }
}

/**
 * @brief Fungsi Utama (Main Loop Headless Daemon) Pengendali NMPC Real-Time pada Mini PC.
 */
int main(int argc, char* argv[]) {
    std::cout << "=========================================================\n";
    std::cout << "  NMPC MINI PC BACKGROUND DAEMON (HEADLESS AUTOMATION)   \n";
    std::cout << "=========================================================\n";

    // Inisialisasi memori dan solver NMPC bawaan C++ MATLAB Coder
    nmpc_kapal_waypoint_initialize();

    double u_prev = 0.0;    // Menyimpan aksi kontrol kemudi sebelumnya (u[k-1]) dalam radian
    int step_counter = 0;   // Menghitung jumlah iterasi loop kontrol yang berjalan

    // Menangani argumen opsional dari CLI (misal: ./mini_pc_way_point_enu.exe COM4)
    std::string manualPort = "";
    if (argc > 1) {
        manualPort = argv[1];
        std::cout << "[MANUAL] Menggunakan port serial spesifik dari argumen CLI: " << manualPort << "\n";
    }

    // Loop Abadi Auto-Reconnect (Headless Daemon)
    while (true) {
        std::string activePort = manualPort;

        // Jika tidak ada argumen manual, jalankan Auto-Detect
        if (activePort.empty()) {
            activePort = autoDetectESP32Port();
        }

        // Jika port belum terdeteksi/gagal ditemukan
        if (activePort.empty()) {
            std::cout << "[WAITING] ESP32 belum terhubung/mati. Mencoba lagi dalam 2 detik...\n";
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue;
        }

        // Membuka koneksi serial utama ke ESP32
        SerialPort esp32Serial(activePort.c_str(), CBR_115200);

        if (!esp32Serial.isConnected()) {
            std::cout << "[WARN] Gagal membuka port " << activePort << ". Mencoba ulang...\n";
            if (!manualPort.empty()) {
                manualPort = ""; // Reset ke mode auto-detect jika port manual gagal dibuka
            }
            std::this_thread::sleep_for(std::chrono::seconds(2));
            continue;
        }

        std::cout << "[RUNNING] Berhasil terhubung ke " << activePort << ". Memulai Loop NMPC...\n";

        std::string line;
        // Loop Pengolahan NMPC selama Serial Masih Terhubung
        while (esp32Serial.isConnected()) {
            if (esp32Serial.readLine(line)) {
                if (!line.empty()) {
                    processNMPCStep(esp32Serial, line, u_prev, step_counter);
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }

        std::cout << "[WARN] Serial terputus! Mengulang auto-detect...\n";
        if (!manualPort.empty()) manualPort = ""; // Reset ke mode auto-detect jika terjadi disconnection
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    // Pembersihan memori solver NMPC sebelum program dihentikan
    nmpc_kapal_waypoint_terminate();
    return 0;
}


