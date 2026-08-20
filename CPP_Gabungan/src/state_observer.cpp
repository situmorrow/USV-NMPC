#include "state_observer.hpp"

namespace usv {

StateObserver::StateObserver(double ship_length, double surge_speed, double yaw_offset_deg)
    : m_L(ship_length),
      m_u0(surge_speed),
      m_yaw_offset_deg(yaw_offset_deg) {}

ShipState StateObserver::update(const TelemetryRow& row,
                                const GuidanceModule& guidance,
                                double u_prev_command) {
  ShipState s;

  s.timestamp = row.timestamp;
  s.lat = row.lat;
  s.lon = row.lon;
  s.raw_yaw_deg = row.yaw;
  s.yaw_rate_dps = row.yaw_rate;
  s.u_surge_mps = m_u0;
  s.v_sway_mps = 0.0; // Dapat diestimasi dari akselerasi lateral jika tersedia sensor

  // 1. Kalibrasi Sudut Yaw (+180 derajat sesuai hasil validasi kapal aktual)
  s.cal_yaw_deg = wrap_to_360(row.yaw + m_yaw_offset_deg);
  s.psi_rad = wrap_to_pi(s.cal_yaw_deg * M_PI / 180.0);
  s.psi_deg = s.psi_rad * 180.0 / M_PI;

  // 2. Transformasi Posisi Real-Time Kapal GPS -> ENU (East, North meter)
  ENUPoint enu = guidance.gps_to_enu(row.lat, row.lon);
  s.x_enu = enu.x;
  s.y_enu = enu.y;

  // 3. Konversi Yaw Rate ke radian/detik
  s.r_rad_s = row.yaw_rate * (M_PI / 180.0);

  // 4. Feedback aktuasi kemudi aktual dari potensiometer servo kapal
  s.rudder_feedback_deg = row.calc_deg_servo_1;
  s.rudder_feedback_rad = row.calc_deg_servo_1 * (M_PI / 180.0);
  s.u_prev_rad = u_prev_command;

  // 5. Pembentukan Vektor State Non-Dimensional untuk Solver NMPC WyNDA:
  //    s[0] = v'   = v / u0
  //    s[1] = r'   = r * (L / u0)
  //    s[2] = x'   = X_enu / L
  //    s[3] = y'   = Y_enu / L
  //    s[4] = psi  = psi_rad
  s.state_nd[0] = s.v_sway_mps / m_u0;
  s.state_nd[1] = s.r_rad_s * (m_L / m_u0);
  s.state_nd[2] = s.x_enu / m_L;
  s.state_nd[3] = s.y_enu / m_L;
  s.state_nd[4] = s.psi_rad;

  m_latest_state = s;
  return s;
}

} // namespace usv
