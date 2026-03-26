#include "core.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>
#include <sstream>

namespace native_l45 {

namespace {

constexpr double kChi2Gate3Dof = 14.16;
constexpr double kMaxPredictGapS = 60.0;
constexpr double kProcessNoiseAccel = 5.0;
constexpr double kMeasurementNoiseM = 200.0;
constexpr double kMaxTrackAgeS = 300.0;
constexpr int kMinTrackUpdates = 2;
constexpr int kMaxTrackHistory = 100;
constexpr double kMpsToKts = 1.9438444924406;
constexpr double kMpsToFpm = 196.85039370079;
constexpr double kStatsInterval = 30.0;
constexpr double kPruneInterval = 60.0;

double monotonic_now() {
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

bool invert3(std::array<std::array<double, 3>, 3> a, std::array<std::array<double, 3>, 3>& inv) {
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      inv[i][j] = (i == j) ? 1.0 : 0.0;
    }
  }
  for (int col = 0; col < 3; ++col) {
    int pivot = col;
    double best = std::abs(a[col][col]);
    for (int row = col + 1; row < 3; ++row) {
      double value = std::abs(a[row][col]);
      if (value > best) {
        best = value;
        pivot = row;
      }
    }
    if (best < 1e-15) {
      return false;
    }
    if (pivot != col) {
      std::swap(a[pivot], a[col]);
      std::swap(inv[pivot], inv[col]);
    }
    double div = a[col][col];
    for (int j = 0; j < 3; ++j) {
      a[col][j] /= div;
      inv[col][j] /= div;
    }
    for (int row = 0; row < 3; ++row) {
      if (row == col) {
        continue;
      }
      double factor = a[row][col];
      if (factor == 0.0) {
        continue;
      }
      for (int j = 0; j < 3; ++j) {
        a[row][j] -= factor * a[col][j];
        inv[row][j] -= factor * inv[col][j];
      }
    }
  }
  return true;
}

std::array<double, 3> mat3_vec(const std::array<std::array<double, 3>, 3>& m, const std::array<double, 3>& v) {
  std::array<double, 3> out{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      out[i] += m[i][j] * v[j];
    }
  }
  return out;
}

std::string stats_with_parse_errors(const std::string& base, int parse_errors) {
  std::string out = base;
  if (!out.empty() && out.back() == '}') {
    out.pop_back();
  }
  out += ",\"parse_errors\":" + std::to_string(parse_errors) + "}";
  return out;
}

}  // namespace

AircraftEKF::AircraftEKF(const Vec3& position, double timestamp_s, double process_accel_, double meas_noise_) {
  x = {position.x, position.y, position.z, 0.0, 0.0, 0.0};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      P.m[i][j] = 0.0;
    }
  }
  P.m[0][0] = meas_noise_ * meas_noise_;
  P.m[1][1] = meas_noise_ * meas_noise_;
  P.m[2][2] = meas_noise_ * meas_noise_;
  P.m[3][3] = 1e6;
  P.m[4][4] = 1e6;
  P.m[5][5] = 1e6;
  last_timestamp_s = timestamp_s;
  updates = 1;
  process_accel = process_accel_;
  meas_noise = meas_noise_;
}

Vec3 AircraftEKF::predict(double timestamp_s) {
  double dt = timestamp_s - last_timestamp_s;
  if (dt <= 0.0) {
    return position();
  }
  if (dt > kMaxPredictGapS) {
    for (int i = 0; i < 6; ++i) {
      P.m[i][i] += 1e6;
    }
    last_timestamp_s = timestamp_s;
    return position();
  }
  std::array<std::array<double, 6>, 6> f{};
  for (int i = 0; i < 6; ++i) {
    f[i][i] = 1.0;
  }
  f[0][3] = dt;
  f[1][4] = dt;
  f[2][5] = dt;
  double q = process_accel * process_accel;
  double dt2 = dt * dt;
  double dt3 = dt2 * dt / 2.0;
  double dt4 = dt2 * dt2 / 4.0;
  std::array<std::array<double, 6>, 6> qmat{};
  for (int i = 0; i < 3; ++i) {
    qmat[i][i] = dt4 * q;
    qmat[i][i + 3] = dt3 * q;
    qmat[i + 3][i] = dt3 * q;
    qmat[i + 3][i + 3] = dt2 * q;
  }
  std::array<double, 6> next{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      next[i] += f[i][j] * x[j];
    }
  }
  std::array<std::array<double, 6>, 6> fp{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      for (int k = 0; k < 6; ++k) {
        fp[i][j] += f[i][k] * P.m[k][j];
      }
    }
  }
  std::array<std::array<double, 6>, 6> next_p{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      for (int k = 0; k < 6; ++k) {
        next_p[i][j] += fp[i][k] * f[j][k];
      }
      next_p[i][j] += qmat[i][j];
    }
  }
  x = next;
  P.m = next_p;
  last_timestamp_s = timestamp_s;
  return position();
}

double AircraftEKF::update(const Vec3& measurement, double timestamp_s, std::optional<double> measurement_noise_m) {
  predict(timestamp_s);
  std::array<double, 3> y = {measurement.x - x[0], measurement.y - x[1], measurement.z - x[2]};
  double noise = measurement_noise_m.value_or(meas_noise);
  std::array<std::array<double, 3>, 3> s{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      s[i][j] = P.m[i][j];
    }
    s[i][i] += noise * noise;
  }
  std::array<std::array<double, 3>, 3> s_inv{};
  if (!invert3(s, s_inv)) {
    return -1.0;
  }
  auto tmp = mat3_vec(s_inv, y);
  double mahal_sq = y[0] * tmp[0] + y[1] * tmp[1] + y[2] * tmp[2];
  // Adaptive gate: tighter for established tracks (>10 updates) to reject outliers
  double effective_gate = updates >= 10 ? 11.34 : kChi2Gate3Dof;  // 99% vs 99.7%
  if (mahal_sq > effective_gate) {
    return -1.0;
  }
  double mahal = std::sqrt(mahal_sq);
  std::array<std::array<double, 3>, 6> k{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int m = 0; m < 3; ++m) {
        k[i][j] += P.m[i][m] * s_inv[m][j];
      }
    }
  }
  for (int i = 0; i < 6; ++i) {
    x[i] += k[i][0] * y[0] + k[i][1] * y[1] + k[i][2] * y[2];
  }
  std::array<std::array<double, 6>, 6> ikh{};
  for (int i = 0; i < 6; ++i) {
    ikh[i][i] = 1.0;
  }
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 3; ++j) {
      ikh[i][j] -= k[i][j];
    }
  }
  std::array<std::array<double, 6>, 6> temp{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      for (int m = 0; m < 6; ++m) {
        temp[i][j] += ikh[i][m] * P.m[m][j];
      }
    }
  }
  std::array<std::array<double, 6>, 6> joseph{};
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      for (int m = 0; m < 6; ++m) {
        joseph[i][j] += temp[i][m] * ikh[j][m];
      }
    }
  }
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) {
      double sum = 0.0;
      for (int m = 0; m < 3; ++m) {
        sum += k[i][m] * k[j][m] * noise * noise;
      }
      joseph[i][j] += sum;
    }
  }
  P.m = joseph;
  ++updates;
  innovations.push_back(mahal);
  return mahal;
}

Vec3 AircraftEKF::position() const {
  return {x[0], x[1], x[2]};
}

Vec3 AircraftEKF::velocity() const {
  return {x[3], x[4], x[5]};
}

double AircraftEKF::median_innovation() const {
  if (innovations.empty()) {
    return 0.0;
  }
  auto sorted = innovations;
  std::sort(sorted.begin(), sorted.end());
  return sorted[sorted.size() / 2];
}

TrackState::TrackState(
    const std::string& icao_,
    const Vec3& position_ecef,
    double timestamp_s,
    double alt_ft,
    int df_type,
    std::optional<std::string> squawk,
    double order)
    : icao(icao_),
      ekf(position_ecef, timestamp_s),
      last_update_order(order),
      creation_order(order),
      last_alt_ft(alt_ft),
      last_df_type(df_type),
      last_squawk(std::move(squawk)) {
  auto [lat, lon, alt] = ecef_to_lla(position_ecef.x, position_ecef.y, position_ecef.z);
  static_cast<void>(alt);
  positions.push_back({lat, lon, alt_ft, timestamp_s});
  timestamps.push_back(timestamp_s);
}

double TrackState::update(
    const Vec3& position_ecef,
    double timestamp_s,
    double alt_ft,
    int df_type,
    std::optional<std::string> squawk,
    std::optional<double> measurement_noise_m,
    double order) {
  double mahal = ekf.update(position_ecef, timestamp_s, measurement_noise_m);
  if (mahal >= 0.0) {
    last_update_order = order;
    last_alt_ft = alt_ft;
    last_df_type = df_type;
    if (squawk) {
      last_squawk = std::move(squawk);
    }
    auto pos = ekf.position();
    auto [lat, lon, alt] = ecef_to_lla(pos.x, pos.y, pos.z);
    static_cast<void>(alt);
    positions.push_back({lat, lon, alt_ft, timestamp_s});
    timestamps.push_back(timestamp_s);
    if (positions.size() > static_cast<std::size_t>(kMaxTrackHistory)) {
      positions.erase(positions.begin(), positions.begin() + (positions.size() - kMaxTrackHistory));
      timestamps.erase(timestamps.begin(), timestamps.begin() + (timestamps.size() - kMaxTrackHistory));
    }
  }
  return mahal;
}

double TrackState::age(double now_order) const {
  return now_order - last_update_order;
}

bool TrackState::is_established() const {
  return ekf.updates >= kMinTrackUpdates;
}

double TrackState::heading_deg() const {
  Vec3 vel = ekf.velocity();
  if (norm(vel) < 1.0) {
    return 0.0;
  }
  Vec3 pos = ekf.position();
  auto [lat, lon, alt] = ecef_to_lla(pos.x, pos.y, pos.z);
  static_cast<void>(alt);
  double lat_r = lat * M_PI / 180.0;
  double lon_r = lon * M_PI / 180.0;
  double sin_lat = std::sin(lat_r);
  double cos_lat = std::cos(lat_r);
  double sin_lon = std::sin(lon_r);
  double cos_lon = std::cos(lon_r);
  double ve = -sin_lon * vel.x + cos_lon * vel.y;
  double vn = -sin_lat * cos_lon * vel.x - sin_lat * sin_lon * vel.y + cos_lat * vel.z;
  return std::fmod(std::atan2(ve, vn) * 180.0 / M_PI + 360.0, 360.0);
}

double TrackState::ground_speed_kts() const {
  Vec3 vel = ekf.velocity();
  Vec3 pos = ekf.position();
  auto [lat, lon, alt] = ecef_to_lla(pos.x, pos.y, pos.z);
  static_cast<void>(alt);
  double lat_r = lat * M_PI / 180.0;
  double lon_r = lon * M_PI / 180.0;
  double sin_lat = std::sin(lat_r);
  double cos_lat = std::cos(lat_r);
  double sin_lon = std::sin(lon_r);
  double cos_lon = std::cos(lon_r);
  double ve = -sin_lon * vel.x + cos_lon * vel.y;
  double vn = -sin_lat * cos_lon * vel.x - sin_lat * sin_lon * vel.y + cos_lat * vel.z;
  return std::sqrt(ve * ve + vn * vn) * kMpsToKts;
}

double TrackState::vertical_rate_fpm() const {
  Vec3 vel = ekf.velocity();
  Vec3 pos = ekf.position();
  auto [lat, lon, alt] = ecef_to_lla(pos.x, pos.y, pos.z);
  static_cast<void>(alt);
  double lat_r = lat * M_PI / 180.0;
  double lon_r = lon * M_PI / 180.0;
  double sin_lat = std::sin(lat_r);
  double cos_lat = std::cos(lat_r);
  double sin_lon = std::sin(lon_r);
  double cos_lon = std::cos(lon_r);
  double vu = cos_lat * cos_lon * vel.x + cos_lat * sin_lon * vel.y + sin_lat * vel.z;
  return vu * kMpsToFpm;
}

TrackOutput TrackState::to_output(const SolveFix& fix) const {
  Vec3 pos = ekf.position();
  auto [lat, lon, alt] = ecef_to_lla(pos.x, pos.y, pos.z);
  static_cast<void>(alt);
  double lat_r = lat * M_PI / 180.0;
  double lon_r = lon * M_PI / 180.0;
  double sin_lat = std::sin(lat_r);
  double cos_lat = std::cos(lat_r);
  double sin_lon = std::sin(lon_r);
  double cos_lon = std::cos(lon_r);
  std::array<std::array<double, 3>, 3> r_enu = {{
      {-sin_lon, cos_lon, 0.0},
      {-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat},
      {cos_lat * cos_lon, cos_lat * sin_lon, sin_lat},
  }};
  std::array<std::array<double, 3>, 3> p_pos{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      p_pos[i][j] = ekf.P.m[i][j];
    }
  }
  std::array<std::array<double, 3>, 3> temp{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        temp[i][j] += r_enu[i][k] * p_pos[k][j];
      }
    }
  }
  std::array<std::array<double, 3>, 3> p_enu{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        p_enu[i][j] += temp[i][k] * r_enu[j][k];
      }
    }
  }
  TrackOutput out;
  out.icao = icao;
  out.lat = round_to(lat, 6);
  out.lon = round_to(lon, 6);
  out.alt_ft = round_to(last_alt_ft, 0);
  out.heading_deg = round_to(heading_deg(), 1);
  out.speed_kts = round_to(ground_speed_kts(), 1);
  out.vrate_fpm = round_to(vertical_rate_fpm(), 0);
  out.track_quality = ekf.updates;
  out.positions_count = static_cast<int>(positions.size());
  out.residual_m = fix.residual_m;
  out.quality_residual_m = fix.quality_residual_m;
  out.gdop = fix.gdop;
  out.num_sensors = fix.num_sensors;
  out.solve_method = fix.solve_method;
  out.timestamp_s = fix.timestamp_s;
  out.timestamp_ns = fix.timestamp_ns;
  out.df_type = last_df_type;
  out.squawk = last_squawk;
  out.raw_msg = fix.raw_msg;
  out.t0_s = fix.t0_s;
  out.cov_matrix = {{{p_enu[0][0], p_enu[0][1]}, {p_enu[1][0], p_enu[1][1]}}};
  return out;
}

std::optional<TrackOutput> TrackManager::process_fix(const SolveFix& fix) {
  ++fixes_received;
  if (fix.icao.empty()) {
    return std::nullopt;
  }
  double alt_m = ft_to_m(fix.alt_ft);
  Vec3 position_ecef = lla_to_ecef(fix.lat, fix.lon, alt_m);
  double ts = static_cast<double>(fix.timestamp_s) + static_cast<double>(fix.timestamp_ns) * 1e-9;
  std::optional<double> meas_noise;
  if (fix.num_sensors == 2 && fix.gdop > 0.0) {
    // Better scaling: lower floor for good-quality 2-sensor fixes
    double noise_floor = fix.quality_residual_m < 50.0 ? 200.0 : 400.0;
    meas_noise = std::min(3000.0, std::max(noise_floor, fix.quality_residual_m * fix.gdop * 2.5));
  } else if (fix.num_sensors == 2) {
    double noise_floor = fix.quality_residual_m < 50.0 ? 250.0 : 400.0;
    meas_noise = std::min(3000.0, std::max(noise_floor, fix.quality_residual_m * 6.0));
  } else if (fix.quality_residual_m > 0.0 && fix.gdop > 0.0) {
    meas_noise = std::min(2000.0, std::max(50.0, fix.quality_residual_m * fix.gdop));
  } else if (fix.quality_residual_m > 0.0) {
    meas_noise = std::min(2000.0, std::max(50.0, fix.quality_residual_m * 2.0));
  }
  double now = monotonic_now();
  auto it = tracks_.find(fix.icao);
  if (it == tracks_.end()) {
    auto [inserted_it, ok] = tracks_.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(fix.icao),
        std::forward_as_tuple(fix.icao, position_ecef, ts, fix.alt_ft, fix.df_type, fix.squawk, now));
    static_cast<void>(ok);
    ++tracks_created;
    ++fixes_accepted;
    return inserted_it->second.to_output(fix);
  }
  double mahal = it->second.update(position_ecef, ts, fix.alt_ft, fix.df_type, fix.squawk, meas_noise, now);
  if (mahal < 0.0) {
    ++fixes_rejected;
    return std::nullopt;
  }
  ++fixes_accepted;
  return it->second.to_output(fix);
}

std::optional<TrackOutput> TrackManager::solve_prediction_aided(const Group& group) {
  if (group.icao.empty()) {
    return std::nullopt;
  }
  auto it = tracks_.find(group.icao);
  if (it == tracks_.end() || !it->second.is_established()) {
    return std::nullopt;
  }
  int n_sensors = static_cast<int>(group.receptions.size());
  if (n_sensors < 2 || !group.altitude_ft) {
    return std::nullopt;
  }
  std::vector<Vec3> sensor_positions(n_sensors);
  std::vector<double> sensor_alts_m(n_sensors);
  std::vector<double> arrival_times(n_sensors);
  int ref_idx = 0;
  long long ref_stamp = std::numeric_limits<long long>::max();
  for (int i = 0; i < n_sensors; ++i) {
    long long stamp = group.receptions[i].timestamp_s * 1000000000LL + group.receptions[i].timestamp_ns;
    if (stamp < ref_stamp) {
      ref_stamp = stamp;
      ref_idx = i;
    }
  }
  std::int64_t ref_timestamp_s = group.receptions[ref_idx].timestamp_s;
  std::int64_t ref_timestamp_ns = group.receptions[ref_idx].timestamp_ns;
  for (int i = 0; i < n_sensors; ++i) {
    sensor_positions[i] = group.receptions[i].sensor_ecef;
    sensor_alts_m[i] = group.receptions[i].alt;
    std::int64_t dt_s = group.receptions[i].timestamp_s - ref_timestamp_s;
    std::int64_t dt_ns = group.receptions[i].timestamp_ns - ref_timestamp_ns;
    arrival_times[i] = static_cast<double>(dt_s) + static_cast<double>(dt_ns) * 1e-9;
  }
  double target_ts = static_cast<double>(ref_timestamp_s) + static_cast<double>(ref_timestamp_ns) * 1e-9;
  Vec3 predicted_ecef = it->second.ekf.predict(target_ts);
  double pred_uncertainty = std::sqrt(it->second.ekf.P.m[0][0] + it->second.ekf.P.m[1][1] + it->second.ekf.P.m[2][2]);
  double pw = clamp(500.0 / std::max(pred_uncertainty, 1.0), 1.0, 12.0);
  auto result = solve_toa(sensor_positions, arrival_times, sensor_alts_m, predicted_ecef, ft_to_m(*group.altitude_ft), predicted_ecef, 50, pw);
  if (!result) {
    return std::nullopt;
  }
  double max_residual = std::min(500.0, std::max(100.0, pred_uncertainty * 0.5));
  if (result->residual_m > max_residual) {
    return std::nullopt;
  }
  double pred_offset = norm(result->position - predicted_ecef);
  double max_offset = std::min(5000.0, std::max(500.0, pred_uncertainty * 3.0));
  if (pred_offset > max_offset) {
    return std::nullopt;
  }
  auto [lat, lon, alt] = ecef_to_lla(result->position.x, result->position.y, result->position.z);
  static_cast<void>(alt);
  SolveFix solved_fix;
  solved_fix.icao = group.icao;
  solved_fix.lat = lat;
  solved_fix.lon = lon;
  solved_fix.alt_ft = *group.altitude_ft;
  solved_fix.residual_m = result->residual_m;
  solved_fix.quality_residual_m = result->objective_residual_m;
  solved_fix.gdop = 0.0;
  solved_fix.num_sensors = n_sensors;
  solved_fix.solve_method = n_sensors == 2 ? "prediction_aided_2sensor" : "prediction_aided_" + std::to_string(n_sensors) + "sensor";
  solved_fix.timestamp_s = ref_timestamp_s;
  solved_fix.timestamp_ns = ref_timestamp_ns;
  solved_fix.df_type = group.df_type;
  solved_fix.squawk = group.squawk;
  solved_fix.raw_msg = group.raw_msg;
  solved_fix.t0_s = result->t0_s;
  --fixes_received;
  return process_fix(solved_fix);
}

std::optional<TrackOutput> TrackManager::process_record(const Layer4Record& record) {
  if (record.is_unsolved_group) {
    ++fixes_received;
    auto result = solve_prediction_aided(record.unsolved_group);
    if (!result) {
      return std::nullopt;
    }
    return result;
  }
  return process_fix(record.fix);
}

int TrackManager::prune_stale() {
  double now = monotonic_now();
  std::vector<std::string> stale;
  stale.reserve(tracks_.size());
  for (const auto& [icao, track] : tracks_) {
    if (track.age(now) > kMaxTrackAgeS) {
      stale.push_back(icao);
    }
  }
  for (const auto& icao : stale) {
    tracks_.erase(icao);
  }
  tracks_pruned += static_cast<int>(stale.size());
  return static_cast<int>(stale.size());
}

std::string TrackManager::stats_json() const {
  int established = 0;
  for (const auto& [icao, track] : tracks_) {
    if (track.is_established()) {
      ++established;
    }
  }
  double accept_rate = fixes_received > 0 ? 100.0 * static_cast<double>(fixes_accepted) / static_cast<double>(fixes_received) : 0.0;
  std::ostringstream os;
  os << '{';
  os << "\"fixes_received\":" << fixes_received;
  os << ",\"fixes_accepted\":" << fixes_accepted;
  os << ",\"fixes_rejected\":" << fixes_rejected;
  os << ",\"tracks_created\":" << tracks_created;
  os << ",\"tracks_pruned\":" << tracks_pruned;
  os << ",\"active_tracks\":" << tracks_.size();
  os << ",\"established_tracks\":" << established;
  os << ",\"accept_rate\":\"" << round_to(accept_rate, 1) << "%\"";
  os << '}';
  return os.str();
}

int run_layer5(std::istream& in, std::ostream& out, std::ostream& log) {
  log << "=== MLAT Track Builder (Layer 5) — Native ===\n";
  log << "Reading JSONL position fixes from stdin\n";
  log << "EKF: 6-state constant-velocity [x, y, z, vx, vy, vz]\n";
  log << "Innovation gate: Chi-squared 3-DOF at 99.7% confidence\n";
  log << "Stats logged every 30s to stderr\n";
  log << "Stale track pruning every 60s\n";
  TrackManager manager;
  int parse_errors = 0;
  double last_stats = monotonic_now();
  double last_prune = last_stats;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) {
      continue;
    }
    Layer4Record record;
    if (!parse_layer4_record_json(line, record)) {
      ++parse_errors;
      continue;
    }
    auto result = manager.process_record(record);
    if (result) {
      out << to_json_track(*result) << '\n';
    }
    double now = monotonic_now();
    if (now - last_prune >= kPruneInterval) {
      int pruned = manager.prune_stale();
      if (pruned > 0) {
        log << "[prune] Removed " << pruned << " stale tracks\n";
      }
      last_prune = now;
    }
    if (now - last_stats >= kStatsInterval) {
      log << "[stats] " << stats_with_parse_errors(manager.stats_json(), parse_errors) << '\n';
      last_stats = now;
    }
  }
  log << "Shutting down. Final stats:\n";
  log << "[stats] " << stats_with_parse_errors(manager.stats_json(), parse_errors) << '\n';
  return 0;
}

}
