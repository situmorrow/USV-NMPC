#pragma once

#include "guidance.hpp"
#include "telemetry_parser.hpp"

#include <cmath>
#include <string>

namespace usv {

/**
 * @brief Helper utility untuk normalisasi dan wrapping sudut
 */
inline double wrap_to_pi(double angle_rad) {
  while (angle_rad > M_PI) angle_rad -= 2.0 * M_PI;
  while (angle_rad < -M_PI) angle_rad += 2.0 * M_PI;
  return angle_rad;
}

inline double wrap_to_360(double angle_deg) {
  angle_deg = std::fmod(angle_deg, 360.0);
  if (angle_deg < 0.0) angle_deg += 360.0;
  return angle_deg;
}

inline double shortest_angular_distance(double from_rad, double to_rad) {
  return std::atan2(std::sin(to_rad - from_rad), std::cos(to_rad - from_rad));
}

/**
 * @brief Struktur data state lengkap kapal (Dimensional dan Non-Dimensional)
 */
struct ShipState {
  // 1. Vektor State Non-Dimensional untuk Solver NMPC (5x1 double array)
  // s = [v', r', x', y', psi]^T
  double state_nd[5]{0.0, 0.0, 0.0, 0.0, 0.0};

  // 2. State Dimensional (Satuan Fisik Nyata)
  double timestamp{0.0};      ///< Waktu telemetri (detik)
  double lat{0.0};            ///< Latitude GPS real-time (derajat)
  double lon{0.0};            ///< Longitude GPS real-time (derajat)
  double x_enu{0.0};          ///< Posisi East X (meter)
  double y_enu{0.0};          ///< Posisi North Y (meter)
  
  double raw_yaw_deg{0.0};    ///< Raw yaw dari sensor IMU kapal (derajat)
  double cal_yaw_deg{0.0};    ///< Yaw terkalibrasi (+offset 180 deg) (derajat)
  double psi_rad{0.0};        ///< Heading kapal sistem ENU (radian)
  double psi_deg{0.0};        ///< Heading kapal sistem ENU (derajat)
  
  double yaw_rate_dps{0.0};   ///< Yaw rate gyro kapal (deg/s)
  double r_rad_s{0.0};        ///< Yaw rate kapal (rad/s)
  double v_sway_mps{0.0};     ///< Kecepatan sway lateral (m/s)
  double u_surge_mps{0.6114}; ///< Kecepatan surge nominal kapal (m/s)

  double rudder_feedback_deg{0.0}; ///< Sudut aktual servo kemudi dari potensiometer (deg)
  double rudder_feedback_rad{0.0}; ///< Sudut aktual servo kemudi (rad)
  double u_prev_rad{0.0};          ///< Sinyal kemudi langkah sebelumnya (rad)
};

/**
 * @brief Estimator State Kapal dan Sinkronisasi Orientasi IMU
 */
class StateObserver {
public:
  /**
   * @brief Konstruktor StateObserver
   * @param ship_length Panjang kapal model (L dalam meter, default 1.0107 m)
   * @param surge_speed Kecepatan surge nominal (u0 dalam m/s, default 0.6114 m/s)
   * @param yaw_offset_deg Offset kalibrasi yaw kapal (default +180.0 derajat sesuai validasi kapal)
   */
  StateObserver(double ship_length = 1.0107,
                double surge_speed = 0.6114,
                double yaw_offset_deg = 180.0);

  /**
   * @brief Update estimasi state kapal dari baris data telemetri serial ESP32
   * @param row Baris telemetri CSV 8 kolom (timestamp, lat, lon, servo1, servo2, yaw, gyro_z, yaw_rate)
   * @param guidance Modul guidance yang menyimpan Home Point acuan koordinat ENU
   * @param u_prev_command Sudut kemudi langkah sebelumnya dalam radian
   * @return ShipState lengkap berisi state_nd[5] siap diinputkan ke solver NMPC
   */
  ShipState update(const TelemetryRow& row,
                   const GuidanceModule& guidance,
                   double u_prev_command = 0.0);

  // Getter & Setter Konfigurasi
  void set_yaw_offset_deg(double offset_deg) { m_yaw_offset_deg = offset_deg; }
  double get_yaw_offset_deg() const { return m_yaw_offset_deg; }

  void set_ship_length(double L) { m_L = L; }
  double get_ship_length() const { return m_L; }

  void set_surge_speed(double u0) { m_u0 = u0; }
  double get_surge_speed() const { return m_u0; }

  const ShipState& get_latest_state() const { return m_latest_state; }

private:
  double m_L;              ///< Panjang kapal [m]
  double m_u0;             ///< Kecepatan surge [m/s]
  double m_yaw_offset_deg; ///< Offset kalibrasi yaw [derajat]

  ShipState m_latest_state;
};

} // namespace usv
