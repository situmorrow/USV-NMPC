#include "guidance.hpp"
#include "nmpc_kapal_waypoint.h"
#include "nmpc_kapal_waypoint_initialize.h"
#include "nmpc_kapal_waypoint_terminate.h"
#include "processor.hpp"
#include "serial_port.hpp"
#include "state_observer.hpp"
#include "telemetry_parser.hpp"

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

namespace {
volatile std::sig_atomic_t g_running = 1;

void handle_signal(int) { g_running = 0; }

void print_usage(const char *program_name) {
  std::cerr
      << "=======================================================================\n"
      << " USV NMPC Waypoint Tracking Control & Serial Bridge (CPP_Gabungan)      \n"
      << "=======================================================================\n\n"
      << "Penggunaan:\n"
      << "  " << program_name << " [opsi]\n\n"
      << "Opsi Komunikasi Serial:\n"
      << "  --port <nama_port>          Port serial (default: COM16 / /dev/ttyUSB0)\n"
      << "  --baud <rate>               Baud rate (default: 115200)\n"
      << "  --timeout <ms>              Timeout baca baris (default: 1000 ms)\n"
      << "  --print <all|csv|wp|nmpc|none> Filter output terminal (default: all)\n"
      << "                            all  = CSV 8 kolom + [WP] + [NMPC]\n"
      << "                            csv  = hanya CSV telemetri\n"
      << "                            wp   = hanya baris [WP]\n"
      << "                            nmpc = hanya status kendali [NMPC]\n"
      << "                            none = hening (quiet / hemat CPU)\n\n"
      << "Opsi Mode Kendali Kemudi (--rudder-mode):\n"
      << "  --rudder-mode <nmpc|zero|yawrate2|demo>\n"
      << "                            nmpc     = Kontrol NMPC Pelacakan Waypoint (Default)\n"
      << "                            zero     = Kemudi netral 0 deg (Safety / Test)\n"
      << "                            yawrate2 = Proporsional yaw_rate * 2 deg (Uji coba)\n"
      << "                            demo     = Operasi matematika field telemetri\n\n"
      << "Opsi Parameter NMPC & Navigasi:\n"
      << "  --yaw-offset <deg>          Offset kalibrasi yaw kapal (default: 180.0)\n"
      << "  --r-tran <meter>            Radius switching waypoint (default: 3.0 m)\n"
      << "  --surge-speed <m/s>         Kecepatan surge nominal u_0 (default: 0.6114 m/s)\n"
      << "  --ship-length <meter>       Panjang kapal model L (default: 1.0107 m)\n"
      << "  --default-test-wp           Muat 4 titik waypoint default kolam uji\n"
      << "  --home <lat,lon>            Set manual Home Point (contoh: -7.28715,112.79600)\n"
      << "  --add-wp <lat,lon>          Tambah titik waypoint manual (dapat diulang)\n\n"
      << "Opsi Demo Mode:\n"
      << "  --op <add|sub|mul|div>      Operasi demo (default: sub)\n"
      << "  --field-a <nama_field>      Field pertama (default: calc_deg_servo_1)\n"
      << "  --field-b <nama_field>      Field kedua (default: calc_deg_servo_2)\n"
      << "  --help                      Tampilkan bantuan ini\n\n"
      << "Perilaku Sistem:\n"
      << "  - Mengirim Heartbeat $HB ke ESP32 setiap 1 detik\n"
      << "  - Menerima dan memproses rute [WP] dinamis via ESP-NOW dari ground station\n"
      << "  - Mengirim kembali perintah timestamp,rudder_deg ke serial @ 10 Hz sinkron\n"
      << "  - Perintah $SHUTDOWN dari dashboard mematikan sistem secara aman\n";
}

bool is_shutdown_line(const std::string &line) {
  return line == "$SHUTDOWN";
}

void request_os_shutdown() {
#ifdef _WIN32
  std::system("shutdown /s /t 5 /c \"USV NMPC: shutdown dari dashboard\"");
#else
  std::system("shutdown -h now");
#endif
}

bool is_waypoint_line(const std::string &line) {
  return line.size() >= 4 && line.compare(0, 4, "[WP]") == 0;
}

bool parse_print_mode(const std::string &value, bool &print_csv, bool &print_wp, bool &print_nmpc) {
  if (value == "all") {
    print_csv = true;
    print_wp = true;
    print_nmpc = true;
    return true;
  }
  if (value == "csv") {
    print_csv = true;
    print_wp = false;
    print_nmpc = false;
    return true;
  }
  if (value == "wp") {
    print_csv = false;
    print_wp = true;
    print_nmpc = false;
    return true;
  }
  if (value == "nmpc") {
    print_csv = false;
    print_wp = false;
    print_nmpc = true;
    return true;
  }
  if (value == "none") {
    print_csv = false;
    print_wp = false;
    print_nmpc = false;
    return true;
  }
  return false;
}

std::string default_port() {
#ifdef _WIN32
  return "COM16";
#else
  return "/dev/ttyUSB0";
#endif
}

std::string format_result_line(double timestamp, double result) {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(3) << timestamp << ","
      << std::setprecision(2) << result;
  return oss.str();
}

double clamp_rudder_deg(double value, double min_deg, double max_deg) {
  return std::max(min_deg, std::min(max_deg, value));
}

bool parse_lat_lon_pair(const std::string &str, double &lat, double &lon) {
  return std::sscanf(str.c_str(), "%lf,%lf", &lat, &lon) == 2;
}

} // namespace

int main(int argc, char **argv) {
  std::string port = default_port();
  uint32_t baud = 115200;
  uint32_t timeout_ms = 1000;
  
  std::string rudder_mode = "nmpc";
  std::string print_mode = "all";
  bool print_csv = true;
  bool print_wp = true;
  bool print_nmpc = true;

  // Parameter Kendali & Fisik USV
  double yaw_offset_deg = 180.0;
  double r_tran = 3.0;
  double surge_speed = 0.6114;
  double ship_length = 1.0107;
  bool load_default_wp = false;

  // Parameter Demo Mode
  MathOp math_op = MathOp::Sub;
  TelemetryField field_a = TelemetryField::CalcDegServo1;
  TelemetryField field_b = TelemetryField::CalcDegServo2;

  // Tempat penyimpanan parameter manual CLI
  std::string manual_home_str;
  std::vector<std::string> manual_wps_str;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      print_usage(argv[0]);
      return 0;
    }
    if (arg == "--port" && i + 1 < argc) {
      port = argv[++i];
      continue;
    }
    if (arg == "--baud" && i + 1 < argc) {
      baud = static_cast<uint32_t>(std::stoul(argv[++i]));
      continue;
    }
    if (arg == "--timeout" && i + 1 < argc) {
      timeout_ms = static_cast<uint32_t>(std::stoul(argv[++i]));
      continue;
    }
    if (arg == "--print" && i + 1 < argc) {
      print_mode = argv[++i];
      if (!parse_print_mode(print_mode, print_csv, print_wp, print_nmpc)) {
        std::cerr << "[ERROR] --print tidak dikenal. Gunakan: all, csv, wp, nmpc, none\n";
        return 1;
      }
      continue;
    }
    if (arg == "--rudder-mode" && i + 1 < argc) {
      rudder_mode = argv[++i];
      if (rudder_mode != "nmpc" && rudder_mode != "zero" &&
          rudder_mode != "yawrate2" && rudder_mode != "demo") {
        std::cerr << "[ERROR] rudder-mode tidak dikenal. Gunakan: nmpc, zero, yawrate2, demo\n";
        return 1;
      }
      continue;
    }
    if (arg == "--yaw-offset" && i + 1 < argc) {
      yaw_offset_deg = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--r-tran" && i + 1 < argc) {
      r_tran = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--surge-speed" && i + 1 < argc) {
      surge_speed = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--ship-length" && i + 1 < argc) {
      ship_length = std::stod(argv[++i]);
      continue;
    }
    if (arg == "--default-test-wp") {
      load_default_wp = true;
      continue;
    }
    if (arg == "--home" && i + 1 < argc) {
      manual_home_str = argv[++i];
      continue;
    }
    if (arg == "--add-wp" && i + 1 < argc) {
      manual_wps_str.push_back(argv[++i]);
      continue;
    }
    if (arg == "--op" && i + 1 < argc) {
      const auto parsed = parse_math_op(argv[++i]);
      if (!parsed) {
        std::cerr << "[ERROR] Operasi demo tidak dikenal\n";
        return 1;
      }
      math_op = *parsed;
      continue;
    }
    if (arg == "--field-a" && i + 1 < argc) {
      const auto parsed = parse_field_name(argv[++i]);
      if (!parsed) {
        std::cerr << "[ERROR] field-a tidak dikenal\n";
        return 1;
      }
      field_a = *parsed;
      continue;
    }
    if (arg == "--field-b" && i + 1 < argc) {
      const auto parsed = parse_field_name(argv[++i]);
      if (!parsed) {
        std::cerr << "[ERROR] field-b tidak dikenal\n";
        return 1;
      }
      field_b = *parsed;
      continue;
    }

    std::cerr << "[ERROR] Argumen tidak dikenal: " << arg << "\n";
    print_usage(argv[0]);
    return 1;
  }

  std::signal(SIGINT, handle_signal);
#ifndef _WIN32
  std::signal(SIGTERM, handle_signal);
#endif

  // Inisialisasi Solver NMPC
  nmpc_kapal_waypoint_initialize();

  // Inisialisasi Modul Guidance & State Observer
  usv::GuidanceModule guidance(ship_length, surge_speed, 0.10, r_tran);
  usv::StateObserver observer(ship_length, surge_speed, yaw_offset_deg);

  // Jika opsi --default-test-wp dipilih (Koordinat uji kolam dari run_nmpc.m)
  if (load_default_wp) {
    guidance.set_home_point(-7.287150, 112.796000);
    guidance.add_waypoint_geo(-7.286750, 112.796000); // WP 1 (0, 44.5 m)
    guidance.add_waypoint_geo(-7.286300, 112.796000); // WP 2 (0, 94.5 m)
    guidance.add_waypoint_geo(-7.286750, 112.796100); // WP 3 (11.0, 44.5 m)
    guidance.add_waypoint_geo(-7.287150, 112.796000); // WP 4 (0, 0 m)
    std::cerr << "[INFO] 4 Titik Waypoint Default Kolam Uji berhasil dimuat.\n";
  }

  // Jika opsi --home manual diberikan
  if (!manual_home_str.empty()) {
    double h_lat = 0.0, h_lon = 0.0;
    if (parse_lat_lon_pair(manual_home_str, h_lat, h_lon)) {
      guidance.set_home_point(h_lat, h_lon);
      std::cerr << "[INFO] Manual Home Point ditetapkan: " << h_lat << ", " << h_lon << "\n";
    }
  }

  // Jika ada opsi --add-wp manual
  for (size_t k = 0; k < manual_wps_str.size(); ++k) {
    double wp_lat = 0.0, wp_lon = 0.0;
    if (parse_lat_lon_pair(manual_wps_str[k], wp_lat, wp_lon)) {
      guidance.add_waypoint_geo(wp_lat, wp_lon);
      std::cerr << "[INFO] Manual WP #" << (k + 1) << " ditambahkan: " << wp_lat << ", " << wp_lon << "\n";
    }
  }

  SerialPort serial;
  if (!serial.open(port, baud)) {
    std::cerr << "[ERROR] Gagal membuka port: " << serial.last_error() << "\n";
    nmpc_kapal_waypoint_terminate();
    return 1;
  }

  std::cerr << "=======================================================================\n"
            << " [INFO] Port Serial : " << port << " @ " << baud << " baud\n"
            << " [INFO] Mode Kendali: " << rudder_mode << "\n"
            << " [INFO] Yaw Offset  : +" << yaw_offset_deg << " deg\n"
            << " [INFO] Radius Tran : " << r_tran << " meter\n"
            << " [INFO] Model Kapal : L=" << ship_length << " m, u0=" << surge_speed << " m/s\n"
            << " [INFO] Output Print: " << print_mode << "\n"
            << " [INFO] Tekan Ctrl+C untuk menghentikan program\n"
            << "=======================================================================\n\n";

  uint64_t valid_lines = 0;
  uint64_t waypoint_lines = 0;
  uint64_t skipped_lines = 0;
  uint64_t write_errors = 0;
  uint64_t nmpc_steps = 0;

  double u_prev_rad = 0.0;
  auto last_hb = std::chrono::steady_clock::now();

  while (g_running) {
    // 1. Heartbeat $HB ke ESP32 setiap 1000 ms
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_hb).count() >= 1000) {
      if (!serial.write_line("$HB")) {
        std::cerr << "[WARN] Gagal kirim heartbeat: " << serial.last_error() << "\n";
      }
      last_hb = now;
    }

    // 2. Baca baris serial dari ESP32
    std::string raw_line;
    if (!serial.read_line(raw_line, timeout_ms)) {
      continue;
    }

    const std::string line = trim_cr(raw_line);
    if (line.empty()) {
      continue;
    }

    if (is_header_line(line)) {
      if (print_csv) {
        std::cout << line << "\n" << std::flush;
      }
      continue;
    }

    // 3. Tangani rute Waypoint [WP] dinamis dari dashboard / remote
    if (is_waypoint_line(line)) {
      ++waypoint_lines;
      guidance.parse_waypoint_line(line);
      if (print_wp) {
        std::cout << line << "\n" << std::flush;
      }
      continue;
    }

    // 4. Tangani perintah $SHUTDOWN dari dashboard
    if (is_shutdown_line(line)) {
      std::cerr << "[INFO] Perintah $SHUTDOWN diterima -- mematikan sistem dalam ~5 detik\n"
                << std::flush;
      if (print_csv || print_wp) {
        std::cout << line << "\n" << std::flush;
      }
      serial.close();
      request_os_shutdown();
      g_running = 0;
      break;
    }

    // 5. Parse data telemetri 8 kolom CSV dari ESP32
    const auto row = parse_telemetry_line(line);
    if (!row) {
      ++skipped_lines;
      continue;
    }

    double rudder_cmd_deg = 0.0;

    if (rudder_mode == "nmpc") {
      // Update State Observer kapal (termasuk kalibrasi yaw +180 deg & posisi ENU)
      usv::ShipState ship_state = observer.update(*row, guidance, u_prev_rad);

      if (guidance.has_home() && !guidance.get_waypoints_enu().empty()) {
        // Update Guidance & Generator Horizon NMPC
        auto g_out = guidance.update_guidance(ship_state.x_enu, ship_state.y_enu);

        if (g_out.is_mission_completed) {
          rudder_cmd_deg = 0.0;
          u_prev_rad = 0.0;
          if (print_nmpc) {
            std::cout << "[NMPC] MISI SELESAI di WP #" << (g_out.active_wp_index + 1)
                      << " | Rudder ditahan 0.0 deg\n" << std::flush;
          }
        } else {
          // Eksekusi NMPC WyNDA Solver
          double u_opt_rad = 0.0;
          double exitflag = 0.0;
          const auto t_start = std::chrono::high_resolution_clock::now();

          nmpc_kapal_waypoint(ship_state.state_nd,
                              u_prev_rad,
                              surge_speed,
                              g_out.x_ref_seq,
                              g_out.y_ref_seq,
                              g_out.psi_ref_seq,
                              &u_opt_rad,
                              &exitflag);

          const auto t_end = std::chrono::high_resolution_clock::now();
          const double calc_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();

          // Fail-Safe: Jika solver gagal konvergen (exitflag <= 0), tahan kemudi sebelumnya
          if (exitflag <= 0.0) {
            rudder_cmd_deg = u_prev_rad * 180.0 / M_PI;
            std::cerr << "[WARN] NMPC solver exitflag=" << exitflag 
                      << " pada t=" << row->timestamp << ", fallback ke u_prev=" << rudder_cmd_deg << " deg\n";
          } else {
            // Batasi sudut kemudi ke batas fisik aman [-35 deg, +35 deg]
            const double u_opt_deg = u_opt_rad * 180.0 / M_PI;
            rudder_cmd_deg = clamp_rudder_deg(u_opt_deg, -35.0, 35.0);
            u_prev_rad = rudder_cmd_deg * M_PI / 180.0;
          }

          ++nmpc_steps;

          if (print_nmpc) {
            std::cout << std::fixed << std::setprecision(2)
                      << "[NMPC] t=" << std::setw(6) << row->timestamp
                      << "s | Pos=(" << std::setw(6) << ship_state.x_enu << "," << std::setw(6) << ship_state.y_enu << ")m"
                      << " | Psi=" << std::setw(6) << ship_state.psi_deg << "°"
                      << " | WP#" << (g_out.active_wp_index + 1)
                      << " (d=" << std::setw(5) << g_out.dist_to_active_wp << "m, Los=" << std::setw(6) << g_out.theta_target_deg << "°)"
                      << " | Rudder=" << std::setw(6) << rudder_cmd_deg << "°"
                      << " | SQP=" << std::setprecision(2) << calc_ms << "ms (flag=" << (int)exitflag << ")\n"
                      << std::flush;
          }
        }
      } else {
        // Fallback: Belum ada Waypoint / Home Point valid
        rudder_cmd_deg = 0.0;
        u_prev_rad = 0.0;
        if (print_nmpc) {
          std::cout << "[NMPC] Menunggu data Home Point & Waypoint dari Ground Station...\n" << std::flush;
        }
      }
    } else if (rudder_mode == "yawrate2") {
      rudder_cmd_deg = clamp_rudder_deg(row->yaw_rate * 2.0, -10.0, 10.0);
    } else if (rudder_mode == "demo") {
      const auto result = compute_result(*row, math_op, field_a, field_b);
      if (!result) {
        ++skipped_lines;
        std::cerr << "[WARN] Hitung result gagal pada t=" << row->timestamp << "\n";
        continue;
      }
      rudder_cmd_deg = *result;
    } else {
      rudder_cmd_deg = 0.0;
    }

    ++valid_lines;
    if (print_csv) {
      std::cout << line << "\n" << std::flush;
    }

    // 6. Format dan kirim perintah sudut kemudi kembali ke ESP32 melalui Serial TX
    const std::string result_line = format_result_line(row->timestamp, rudder_cmd_deg);
    if (!serial.write_line(result_line)) {
      ++write_errors;
      std::cerr << "[ERROR] Gagal tulis ke serial: " << serial.last_error()
                << " | line=" << result_line << "\n";
    }
  }

  // Bersihkan Alokasi Memori Solver NMPC
  nmpc_kapal_waypoint_terminate();

  std::cerr << "\n=======================================================================\n"
            << " [INFO] Program Selesai.\n"
            << "   -> Baris Valid Diterima : " << valid_lines << "\n"
            << "   -> Baris [WP] Diproses  : " << waypoint_lines << "\n"
            << "   -> Iterasi NMPC SQP     : " << nmpc_steps << "\n"
            << "   -> Baris Dilewati       : " << skipped_lines << "\n"
            << "   -> Gagal Tulis Serial   : " << write_errors << "\n"
            << "=======================================================================\n";

  return 0;
}
