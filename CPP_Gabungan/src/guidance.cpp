#include "guidance.hpp"

#include <algorithm>
#include <cstdio>
#include <sstream>

namespace usv {

GuidanceModule::GuidanceModule(double ship_length,
                               double surge_speed,
                               double sample_time,
                               double r_transition)
    : m_L(ship_length),
      m_u0(surge_speed),
      m_T_sim(sample_time),
      m_r_tran(r_transition) {}

void GuidanceModule::set_home_point(double lat, double lon) {
  m_home_geo = GeoPoint(lat, lon);
  m_home_valid = true;
  recalculate_waypoints_enu();
}

void GuidanceModule::set_waypoints_geo(const std::vector<GeoPoint>& wps) {
  m_waypoints_geo = wps;
  m_active_wp_index = 0;
  recalculate_waypoints_enu();
}

void GuidanceModule::add_waypoint_geo(double lat, double lon) {
  m_waypoints_geo.emplace_back(lat, lon);
  recalculate_waypoints_enu();
}

void GuidanceModule::clear_waypoints() {
  m_waypoints_geo.clear();
  m_waypoints_enu.clear();
  m_active_wp_index = 0;
}

ENUPoint GuidanceModule::gps_to_enu(double lat, double lon) const {
  if (!m_home_valid) {
    return ENUPoint(0.0, 0.0);
  }
  const double deg2rad = M_PI / 180.0;
  const double dLat = (lat - m_home_geo.lat) * deg2rad;
  const double dLon = (lon - m_home_geo.lon) * deg2rad;
  const double lat0_rad = m_home_geo.lat * deg2rad;

  const double y_north = R_EARTH * dLat;
  const double x_east = R_EARTH * std::cos(lat0_rad) * dLon;

  return ENUPoint(x_east, y_north);
}

GeoPoint GuidanceModule::enu_to_gps(double x_east, double y_north) const {
  if (!m_home_valid) {
    return GeoPoint(0.0, 0.0);
  }
  const double rad2deg = 180.0 / M_PI;
  const double deg2rad = M_PI / 180.0;
  const double lat0_rad = m_home_geo.lat * deg2rad;

  const double dLat = y_north / R_EARTH;
  const double dLon = x_east / (R_EARTH * std::cos(lat0_rad));

  return GeoPoint(m_home_geo.lat + dLat * rad2deg, m_home_geo.lon + dLon * rad2deg);
}

void GuidanceModule::recalculate_waypoints_enu() {
  m_waypoints_enu.clear();
  if (!m_home_valid) {
    return;
  }
  m_waypoints_enu.reserve(m_waypoints_geo.size());
  for (const auto& wp : m_waypoints_geo) {
    m_waypoints_enu.push_back(gps_to_enu(wp.lat, wp.lon));
  }
}

bool GuidanceModule::parse_waypoint_line(const std::string& line) {
  if (line.size() < 4 || line.compare(0, 4, "[WP]") != 0) {
    return false;
  }

  // Format 1: [WP] msg_type=0xA1 home_valid=1 count=4
  if (line.find("count=") != std::string::npos) {
    int home_valid = 0;
    int count = 0;
    if (std::sscanf(line.c_str(), "[WP] msg_type=%*x home_valid=%d count=%d", &home_valid, &count) >= 1) {
      m_expected_count = count;
      m_waypoints_geo.clear();
      m_waypoints_enu.clear();
      m_active_wp_index = 0;
      return true;
    }
  }

  // Format 2: [WP] Home: -7.287150, 112.796000
  if (line.find("Home:") != std::string::npos) {
    double lat = 0.0;
    double lon = 0.0;
    if (std::sscanf(line.c_str(), "[WP] Home: %lf, %lf", &lat, &lon) == 2) {
      set_home_point(lat, lon);
      return true;
    }
  }

  // Format 3: [WP] #1: -7.286750, 112.796000
  if (line.find('#') != std::string::npos && line.find(':') != std::string::npos) {
    int wp_idx = 0;
    double lat = 0.0;
    double lon = 0.0;
    if (std::sscanf(line.c_str(), "[WP] #%d: %lf, %lf", &wp_idx, &lat, &lon) == 3) {
      add_waypoint_geo(lat, lon);
      return true;
    }
  }

  return false;
}

GuidanceOutput GuidanceModule::update_guidance(double x_ship, double y_ship) {
  GuidanceOutput out;
  out.total_waypoints = m_waypoints_enu.size();

  if (m_waypoints_enu.empty()) {
    out.has_valid_waypoints = false;
    out.is_mission_completed = false;
    return out;
  }

  out.has_valid_waypoints = true;

  // Batasi index agar selalu dalam rentang valid
  if (m_active_wp_index >= m_waypoints_enu.size()) {
    m_active_wp_index = m_waypoints_enu.size() - 1;
  }

  ENUPoint target_wp = m_waypoints_enu[m_active_wp_index];
  double dist_to_wp = std::hypot(target_wp.x - x_ship, target_wp.y - y_ship);

  // Logika Switching Radius r_tran (3.0 m)
  if (dist_to_wp <= m_r_tran && (m_active_wp_index + 1) < m_waypoints_enu.size()) {
    m_active_wp_index++;
    target_wp = m_waypoints_enu[m_active_wp_index];
    dist_to_wp = std::hypot(target_wp.x - x_ship, target_wp.y - y_ship);
  }

  // Cek apakah mencapai waypoint terakhir
  if (m_active_wp_index == m_waypoints_enu.size() - 1 && dist_to_wp <= m_r_tran) {
    out.is_mission_completed = true;
  } else {
    out.is_mission_completed = false;
  }

  // Hitung sudut target line-of-sight (theta_target)
  const double theta_target = std::atan2(target_wp.y - y_ship, target_wp.x - x_ship);

  out.active_wp_index = m_active_wp_index;
  out.dist_to_active_wp = dist_to_wp;
  out.theta_target_rad = theta_target;
  out.theta_target_deg = theta_target * 180.0 / M_PI;

  // Bangun sekuens referensi horizon N-Langkah (N = 20)
  for (int h = 0; h < 20; ++h) {
    const double step_idx = static_cast<double>(h + 1);
    const double dist_step = step_idx * m_u0 * m_T_sim;
    const double x_ref_dim = x_ship + dist_step * std::cos(theta_target);
    const double y_ref_dim = y_ship + dist_step * std::sin(theta_target);

    out.x_ref_seq[h] = x_ref_dim / m_L;
    out.y_ref_seq[h] = y_ref_dim / m_L;
    out.psi_ref_seq[h] = theta_target;
  }

  return out;
}

GuidanceOutput GuidanceModule::update_guidance_gps(double lat_ship, double lon_ship) {
  const ENUPoint ship_enu = gps_to_enu(lat_ship, lon_ship);
  return update_guidance(ship_enu.x, ship_enu.y);
}

} // namespace usv
