#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace usv {

/**
 * @brief Konstanta jari-jari bumi standar WGS84 (meter)
 */
constexpr double R_EARTH = 6371000.0;

/**
 * @brief Struktur koordinat geografis (Latitude, Longitude)
 */
struct GeoPoint {
  double lat{0.0}; ///< Latitude dalam derajat desimal
  double lon{0.0}; ///< Longitude dalam derajat desimal

  GeoPoint() = default;
  GeoPoint(double l_lat, double l_lon) : lat(l_lat), lon(l_lon) {}
};

/**
 * @brief Struktur koordinat bidang datar ENU (East, North) dalam meter
 */
struct ENUPoint {
  double x{0.0}; ///< Sumbu East / Timur (meter)
  double y{0.0}; ///< Sumbu North / Utara (meter)

  ENUPoint() = default;
  ENUPoint(double l_x, double l_y) : x(l_x), y(l_y) {}
};

/**
 * @brief Hasil perhitungan panduan lintasan horizon NMPC per langkah
 */
struct GuidanceOutput {
  size_t active_wp_index{0};       ///< Index waypoint aktif saat ini (0-indexed)
  size_t total_waypoints{0};        ///< Jumlah total waypoint
  double dist_to_active_wp{0.0};    ///< Jarak kapal ke waypoint aktif (meter)
  double theta_target_rad{0.0};     ///< Sudut target line-of-sight (radian)
  double theta_target_deg{0.0};     ///< Sudut target line-of-sight (derajat)
  bool is_mission_completed{false}; ///< True jika kapal sudah mencapai waypoint terakhir
  bool has_valid_waypoints{false};  ///< True jika waypoint sudah dimuat

  double x_ref_seq[20]{};           ///< Sekuens referensi X horizon (non-dimensional)
  double y_ref_seq[20]{};           ///< Sekuens referensi Y horizon (non-dimensional)
  double psi_ref_seq[20]{};         ///< Sekuens referensi Psi horizon (radian)
};

/**
 * @brief Modul Guidance & Transformasi Koordinat untuk Autonomous Ship NMPC
 */
class GuidanceModule {
public:
  /**
   * @brief Konstruktor modul guidance
   * @param ship_length Panjang model kapal (L dalam meter, default 1.0107 m)
   * @param surge_speed Kecepatan surge nominal (u_0 dalam m/s, default 0.6114 m/s)
   * @param sample_time Waktu sampling (T_sim dalam detik, default 0.10 s)
   * @param r_transition Radius switching transisi antar waypoint (meter, default 3.0 m)
   */
  GuidanceModule(double ship_length = 1.0107,
                 double surge_speed = 0.6114,
                 double sample_time = 0.10,
                 double r_transition = 3.0);

  /**
   * @brief Menetapkan Home Point (titik acuan konversi ENU)
   */
  void set_home_point(double lat, double lon);

  /**
   * @brief Menetapkan daftar waypoints geografis (Lat, Lon)
   */
  void set_waypoints_geo(const std::vector<GeoPoint>& wps);

  /**
   * @brief Menambah 1 titik waypoint geografis
   */
  void add_waypoint_geo(double lat, double lon);

  /**
   * @brief Mengosongkan daftar waypoint
   */
  void clear_waypoints();

  /**
   * @brief Mengubah koordinat GPS (Lat, Lon) menjadi koordinat ENU (East, North) dalam meter
   *        relatif terhadap Home Point.
   */
  ENUPoint gps_to_enu(double lat, double lon) const;

  /**
   * @brief Mengubah koordinat ENU (East, North) dalam meter menjadi GPS (Lat, Lon)
   *        relatif terhadap Home Point.
   */
  GeoPoint enu_to_gps(double x_east, double y_north) const;

  /**
   * @brief Parse baris serial pesan [WP] dari ESP32-S3 Remote-Side.
   * Format yang didukung:
   *   "[WP] msg_type=0xA1 home_valid=1 count=4"
   *   "[WP] Home: -7.287150, 112.796000"
   *   "[WP] #1: -7.286750, 112.796000"
   * @return true jika baris berhasil diparse sebagai komponen waypoint
   */
  bool parse_waypoint_line(const std::string& line);

  /**
   * @brief Update panduan rute dan generate sekuens horizon NMPC 20-langkah
   * @param x_ship Posisi kapal saat ini di sumbu East (meter)
   * @param y_ship Posisi kapal saat ini di sumbu North (meter)
   * @return GuidanceOutput yang siap diinputkan ke solver NMPC
   */
  GuidanceOutput update_guidance(double x_ship, double y_ship);

  /**
   * @brief Update panduan rute langsung dari posisi GPS telemetri real-time kapal
   * @param lat_ship Latitude kapal saat ini (derajat)
   * @param lon_ship Longitude kapal saat ini (derajat)
   * @return GuidanceOutput yang siap diinputkan ke solver NMPC
   */
  GuidanceOutput update_guidance_gps(double lat_ship, double lon_ship);

  // Getter & Setter parameter
  bool has_home() const { return m_home_valid; }
  GeoPoint get_home_point() const { return m_home_geo; }
  const std::vector<GeoPoint>& get_waypoints_geo() const { return m_waypoints_geo; }
  const std::vector<ENUPoint>& get_waypoints_enu() const { return m_waypoints_enu; }
  size_t get_active_wp_index() const { return m_active_wp_index; }
  void reset_mission() { m_active_wp_index = 0; }

  void set_r_tran(double r_tran) { m_r_tran = r_tran; }
  double get_r_tran() const { return m_r_tran; }

  void set_surge_speed(double u0) { m_u0 = u0; }
  double get_surge_speed() const { return m_u0; }

  void set_ship_length(double L) { m_L = L; }
  double get_ship_length() const { return m_L; }

private:
  void recalculate_waypoints_enu();

  double m_L;                 ///< Panjang kapal [m]
  double m_u0;                ///< Kecepatan surge nominal [m/s]
  double m_T_sim;             ///< Waktu sampling per iterasi [s]
  double m_r_tran;            ///< Switching radius waypoint [m]

  bool m_home_valid{false};
  GeoPoint m_home_geo{0.0, 0.0};
  std::vector<GeoPoint> m_waypoints_geo;
  std::vector<ENUPoint> m_waypoints_enu;

  size_t m_active_wp_index{0};
  int m_expected_count{0};
};

} // namespace usv
