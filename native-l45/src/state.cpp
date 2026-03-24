#include "core.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <queue>
#include <sstream>

namespace native_l45 {

namespace {

constexpr double kOverrideMatchThresholdDeg = 0.5;
constexpr double kMaxCacheAge = 120.0;
constexpr std::size_t kMaxCacheSize = 5000;
constexpr double kMaxAircraftSpeedMps = 1030.0;
constexpr double kMaxAircraftAccelMps2 = 50.0;
constexpr int kMinSyncPoints = 3;
constexpr double kOutlierSigma = 3.5;
constexpr int kMaxConsecutiveOutliers = 5;
constexpr double kMaxClockOffset = 0.5;
constexpr double kClockR = 25e-12;
constexpr double kClockQDrift = 1e-18;

}  // namespace

SensorOverrideMap::SensorOverrideMap(std::vector<OverrideEntry> overrides) : overrides_(std::move(overrides)) {}

const OverrideEntry* SensorOverrideMap::lookup(std::int64_t sensor_id, double stream_lat, double stream_lon) {
  auto it = sensor_map_.find(sensor_id);
  if (it != sensor_map_.end()) {
    return &overrides_[it->second];
  }
  if (rejected_.contains(sensor_id)) {
    return nullptr;
  }
  const OverrideEntry* best = nullptr;
  std::size_t best_idx = 0;
  double best_dist = kOverrideMatchThresholdDeg;
  for (std::size_t i = 0; i < overrides_.size(); ++i) {
    double dist = std::hypot(stream_lat - overrides_[i].lat, stream_lon - overrides_[i].lon);
    if (dist < best_dist) {
      best_dist = dist;
      best = &overrides_[i];
      best_idx = i;
    }
  }
  if (best == nullptr) {
    rejected_[sensor_id] = true;
    return nullptr;
  }
  sensor_map_[sensor_id] = best_idx;
  ++matched_count_;
  return best;
}

bool SensorOverrideMap::empty() const {
  return overrides_.empty();
}

std::string SensorOverrideMap::stats_json() const {
  std::ostringstream os;
  os << '{';
  os << "\"matched_sensors\":" << matched_count_;
  os << ",\"rejected_sensors\":" << rejected_.size();
  os << ",\"sensor_names\":{";
  bool first = true;
  for (const auto& [sensor_id, idx] : sensor_map_) {
    if (!first) {
      os << ',';
    }
    first = false;
    os << '"' << sensor_id << "\":\"" << json_escape(overrides_[idx].name) << '"';
  }
  os << "}}";
  return os.str();
}

std::optional<SensorOverrideMap> load_location_overrides() {
  std::vector<std::filesystem::path> candidates = {
      std::filesystem::current_path() / "data-pipe" / "location-overrides.txt",
      std::filesystem::current_path() / "location-override(2).json",
      std::filesystem::current_path() / ".." / "data-pipe" / "location-overrides.txt",
      std::filesystem::current_path() / ".." / "location-override(2).json",
  };
  for (const auto& path : candidates) {
    if (!std::filesystem::exists(path)) {
      continue;
    }
    std::ifstream in(path);
    if (!in) {
      continue;
    }
    std::stringstream buffer;
    buffer << in.rdbuf();
    try {
      std::string text = buffer.str();
      JsonParser parser(text);
      JsonValue root = parser.parse();
      if (!root.is_array()) {
        continue;
      }
      std::vector<OverrideEntry> overrides;
      for (const auto& item : root.as_array()) {
        if (!item.is_object()) {
          continue;
        }
        const auto& object = item.as_object();
        auto public_key = json_get_string_optional(object, "public_key");
        auto lat = json_get_number_optional(object, "lat");
        auto lon = json_get_number_optional(object, "lon");
        auto alt = json_get_number_optional(object, "alt");
        auto name = json_get_string_optional(object, "name");
        if (!public_key || !lat || !lon || !alt || !name) {
          continue;
        }
        overrides.push_back(OverrideEntry{*public_key, *lat, *lon, *alt, *name});
      }
      return SensorOverrideMap(std::move(overrides));
    } catch (...) {
      continue;
    }
  }
  return std::nullopt;
}

std::vector<Reception> apply_overrides(const std::vector<Reception>& receptions, SensorOverrideMap& override_map) {
  std::vector<Reception> corrected;
  corrected.reserve(receptions.size());
  for (const auto& rec : receptions) {
    const OverrideEntry* override = override_map.lookup(rec.sensor_id, rec.lat, rec.lon);
    if (override == nullptr) {
      continue;
    }
    Reception next = rec;
    next.lat = override->lat;
    next.lon = override->lon;
    next.alt = override->alt;
    next.sensor_ecef = sensor_lla_to_ecef(next.lat, next.lon, next.alt);
    corrected.push_back(next);
  }
  return corrected;
}

Vec3 CachedPosition::predict(double target_timestamp) const {
  double dt = target_timestamp - timestamp;
  if (velocity_ecef && dt > 0.0 && dt < 60.0) {
    return ecef + (*velocity_ecef) * dt;
  }
  return ecef;
}

bool CachedPosition::is_physically_consistent(const Vec3& new_ecef, double new_timestamp, double new_residual_m) const {
  double dt = std::abs(new_timestamp - timestamp);
  if (dt < 0.001) {
    return true;
  }
  double distance = norm(new_ecef - ecef);
  double position_slack = 2.0 * (residual_m + new_residual_m);
  double effective_distance = std::max(0.0, distance - position_slack);
  double implied_speed = effective_distance / dt;
  if (implied_speed > kMaxAircraftSpeedMps) {
    return false;
  }
  if (velocity_ecef && dt > 0.1) {
    Vec3 predicted = ecef + (*velocity_ecef) * (new_timestamp - timestamp);
    double prediction_error = norm(new_ecef - predicted);
    double max_deviation = 0.5 * kMaxAircraftAccelMps2 * dt * dt;
    if (prediction_error > std::max(max_deviation + position_slack, 5000.0)) {
      return false;
    }
  }
  return true;
}

std::optional<CachedPosition> PositionCache::get(const std::string& icao, std::optional<double> target_timestamp) {
  auto it = cache_.find(icao);
  if (it == cache_.end()) {
    ++misses_;
    return std::nullopt;
  }
  if (target_timestamp) {
    double age = std::abs(*target_timestamp - it->second.timestamp);
    if (age > kMaxCacheAge) {
      ++misses_;
      return std::nullopt;
    }
  }
  ++hits_;
  return it->second;
}

void PositionCache::put(const std::string& icao, const Vec3& ecef, double lat, double lon, double alt_m, double timestamp, double residual_m) {
  std::optional<Vec3> velocity;
  int solve_count = 1;
  auto it = cache_.find(icao);
  if (it != cache_.end()) {
    double dt = timestamp - it->second.timestamp;
    if (dt > 0.1 && dt < 60.0) {
      velocity = (ecef - it->second.ecef) / dt;
    }
    solve_count = it->second.solve_count + 1;
  }
  CachedPosition entry;
  entry.ecef = ecef;
  entry.lat = lat;
  entry.lon = lon;
  entry.alt_m = alt_m;
  entry.timestamp = timestamp;
  entry.velocity_ecef = velocity;
  entry.residual_m = residual_m;
  entry.solve_count = solve_count;
  entry.last_order = static_cast<double>(++order_counter_);
  cache_[icao] = entry;
  if (cache_.size() > kMaxCacheSize) {
    prune();
  }
}

void PositionCache::update_velocity_from_adsb(const std::string& icao, double ew_mps, double ns_mps, double vrate_mps, double timestamp) {
  auto it = cache_.find(icao);
  if (it == cache_.end()) {
    return;
  }
  double lat_r = it->second.lat * M_PI / 180.0;
  double lon_r = it->second.lon * M_PI / 180.0;
  double sin_lat = std::sin(lat_r);
  double cos_lat = std::cos(lat_r);
  double sin_lon = std::sin(lon_r);
  double cos_lon = std::cos(lon_r);
  double vx = -sin_lon * ew_mps - sin_lat * cos_lon * ns_mps + cos_lat * cos_lon * vrate_mps;
  double vy = cos_lon * ew_mps - sin_lat * sin_lon * ns_mps + cos_lat * sin_lon * vrate_mps;
  double vz = cos_lat * ns_mps + sin_lat * vrate_mps;
  it->second.velocity_ecef = Vec3{vx, vy, vz};
  it->second.timestamp = timestamp;
  it->second.last_order = static_cast<double>(++order_counter_);
}

void PositionCache::prune() {
  std::vector<std::pair<std::string, double>> entries;
  entries.reserve(cache_.size());
  for (const auto& [icao, cached] : cache_) {
    entries.emplace_back(icao, cached.last_order);
  }
  std::sort(entries.begin(), entries.end(), [](const auto& a, const auto& b) { return a.second < b.second; });
  std::size_t to_remove = entries.size() / 2;
  for (std::size_t i = 0; i < to_remove; ++i) {
    cache_.erase(entries[i].first);
  }
}

std::string PositionCache::stats_json() const {
  std::ostringstream os;
  os << '{';
  os << "\"cached_aircraft\":" << cache_.size();
  os << ",\"hits\":" << hits_;
  os << ",\"misses\":" << misses_;
  double denom = static_cast<double>(hits_ + misses_);
  double hit_rate = denom > 0.0 ? 100.0 * static_cast<double>(hits_) / denom : 0.0;
  os << ",\"hit_rate\":\"" << round_to(hit_rate, 1) << "%\"";
  int multi_solve = 0;
  for (const auto& [icao, cached] : cache_) {
    if (cached.solve_count > 1) {
      ++multi_solve;
    }
  }
  os << ",\"multi_solve\":" << multi_solve;
  os << '}';
  return os.str();
}

std::size_t PositionCache::size() const {
  return cache_.size();
}

ClockPairing::ClockPairing(std::int64_t sensor_a_, std::int64_t sensor_b_) : sensor_a(sensor_a_), sensor_b(sensor_b_) {}

bool ClockPairing::update(double measured_offset, double now) {
  double dt = last_update > 0.0 ? now - last_update : 0.0;
  if (n == 0) {
    offset = measured_offset;
    drift = 0.0;
    p00 = kClockR * 100.0;
    p01 = 0.0;
    p11 = 1e-12;
    variance = p00;
    n = 1;
    last_update = now;
    return true;
  }
  offset += drift * dt;
  double old_p00 = p00;
  double old_p01 = p01;
  double old_p11 = p11;
  p00 = old_p00 + 2.0 * dt * old_p01 + dt * dt * old_p11 + kClockQDrift * dt * dt * dt / 3.0;
  p01 = old_p01 + dt * old_p11 + kClockQDrift * dt * dt / 2.0;
  p11 = old_p11 + kClockQDrift * dt;
  double innovation = measured_offset - offset;
  double s = p00 + kClockR;
  if (valid && n >= kMinSyncPoints) {
    if (std::abs(innovation) > kOutlierSigma * std::sqrt(s)) {
      ++consecutive_outliers;
      if (consecutive_outliers >= kMaxConsecutiveOutliers) {
        n = 0;
        offset = measured_offset;
        drift = 0.0;
        p00 = kClockR * 100.0;
        p01 = 0.0;
        p11 = 1e-12;
        variance = p00;
        valid = false;
        n = 1;
        last_update = now;
        consecutive_outliers = 0;
        return true;
      }
      variance = p00;
      last_update = now;
      return false;
    }
  }
  double k0 = p00 / s;
  double k1 = p01 / s;
  offset += k0 * innovation;
  drift += k1 * innovation;
  double a = 1.0 - k0;
  double new_p00 = a * a * p00 + k0 * k0 * kClockR;
  double new_p01 = a * (-k1 * p00 + p01) + k0 * k1 * kClockR;
  double new_p11 = k1 * k1 * p00 - 2.0 * k1 * p01 + p11 + k1 * k1 * kClockR;
  p00 = new_p00;
  p01 = new_p01;
  p11 = new_p11;
  variance = p00;
  ++n;
  valid = n >= kMinSyncPoints;
  last_update = now;
  consecutive_outliers = 0;
  return true;
}

double ClockPairing::predict(double now) const {
  if (!valid) {
    return offset;
  }
  double dt = now - last_update;
  return offset + drift * dt;
}

std::uint64_t ClockCalibrator::pair_key(std::int64_t a, std::int64_t b) {
  std::uint64_t lo = static_cast<std::uint64_t>(std::min(a, b));
  std::uint64_t hi = static_cast<std::uint64_t>(std::max(a, b));
  return (lo << 32) ^ hi;
}

ClockPairing& ClockCalibrator::get_pairing(std::int64_t a, std::int64_t b) {
  std::uint64_t key = pair_key(a, b);
  auto it = pairings.find(key);
  if (it == pairings.end()) {
    it = pairings.emplace(key, ClockPairing(std::min(a, b), std::max(a, b))).first;
  }
  return it->second;
}

void ClockCalibrator::process_adsb_reference(const Vec3& aircraft_ecef, const std::vector<Reception>& receptions, double now) {
  int n = static_cast<int>(receptions.size());
  if (n < 2) {
    return;
  }
  std::vector<std::pair<std::int64_t, double>> corrected;
  corrected.reserve(receptions.size());
  for (const auto& rec : receptions) {
    double distance = norm(aircraft_ecef - rec.sensor_ecef);
    double delay = distance / kAirC;
    double t_raw = static_cast<double>(rec.timestamp_s) + static_cast<double>(rec.timestamp_ns) * 1e-9;
    corrected.emplace_back(rec.sensor_id, t_raw - delay);
  }
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      auto [sa, ta] = corrected[i];
      auto [sb, tb] = corrected[j];
      double measured_offset = ta - tb;
      ++sync_points_total;
      ClockPairing& pairing = get_pairing(sa, sb);
      if (sa > sb) {
        measured_offset = -measured_offset;
      }
      if (pairing.update(measured_offset, now)) {
        ++sync_points_accepted;
      }
    }
  }
  calibrated_pairs = 0;
  for (const auto& [key, pairing] : pairings) {
    if (pairing.valid) {
      ++calibrated_pairs;
    }
  }
}

std::vector<Reception> ClockCalibrator::correct_timestamps(const std::vector<Reception>& receptions, double now) {
  if (receptions.size() < 2) {
    return receptions;
  }
  std::vector<Reception> corrected = receptions;
  int n = static_cast<int>(corrected.size());
  struct Edge {
    double variance = 0.0;
    int src = 0;
    int dst = 0;
    double corr = 0.0;
  };
  std::vector<Edge> edges;
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      auto key = pair_key(corrected[i].sensor_id, corrected[j].sensor_id);
      auto it = pairings.find(key);
      if (it == pairings.end() || !it->second.valid) {
        continue;
      }
      double offset = it->second.predict(now);
      double variance = it->second.variance > 0.0 ? it->second.variance : 1e-20;
      double correction = corrected[i].sensor_id == it->second.sensor_a ? offset : -offset;
      edges.push_back({variance, i, j, correction});
    }
  }
  if (edges.empty()) {
    return corrected;
  }
  std::vector<int> edge_count(n, 0);
  std::vector<std::vector<std::tuple<double, int, double>>> adj(n);
  for (const auto& edge : edges) {
    adj[edge.src].push_back({edge.variance, edge.dst, edge.corr});
    adj[edge.dst].push_back({edge.variance, edge.src, -edge.corr});
    ++edge_count[edge.src];
    ++edge_count[edge.dst];
  }
  int ref_node = static_cast<int>(std::max_element(edge_count.begin(), edge_count.end()) - edge_count.begin());
  using HeapNode = std::tuple<double, int, int, double>;
  std::priority_queue<HeapNode, std::vector<HeapNode>, std::greater<HeapNode>> heap;
  std::vector<bool> visited(n, false);
  std::vector<double> correction(n, 0.0);
  visited[ref_node] = true;
  for (const auto& [var, neighbor, corr] : adj[ref_node]) {
    heap.push({var, ref_node, neighbor, corr});
  }
  while (!heap.empty()) {
    auto [var, src, dst, corr] = heap.top();
    heap.pop();
    if (visited[dst]) {
      continue;
    }
    visited[dst] = true;
    correction[dst] = correction[src] + corr;
    for (const auto& [next_var, next_node, next_corr] : adj[dst]) {
      if (!visited[next_node]) {
        heap.push({next_var, dst, next_node, next_corr});
      }
    }
  }
  for (int i = 0; i < n; ++i) {
    if (i == ref_node || !visited[i]) {
      continue;
    }
    double corr = correction[i];
    if (std::abs(corr) > kMaxClockOffset) {
      continue;
    }
    std::int64_t correction_ns = static_cast<std::int64_t>(std::llround(corr * 1e9));
    corrected[i].timestamp_ns += correction_ns;
    while (corrected[i].timestamp_ns >= 1000000000) {
      corrected[i].timestamp_ns -= 1000000000;
      ++corrected[i].timestamp_s;
    }
    while (corrected[i].timestamp_ns < 0) {
      corrected[i].timestamp_ns += 1000000000;
      --corrected[i].timestamp_s;
    }
  }
  ++groups_corrected;
  return corrected;
}

bool ClockCalibrator::has_any_calibration(const std::vector<Reception>& receptions) const {
  for (std::size_t i = 0; i < receptions.size(); ++i) {
    for (std::size_t j = i + 1; j < receptions.size(); ++j) {
      auto key = pair_key(receptions[i].sensor_id, receptions[j].sensor_id);
      auto it = pairings.find(key);
      if (it != pairings.end() && it->second.valid) {
        return true;
      }
    }
  }
  return false;
}

std::string ClockCalibrator::stats_json() const {
  std::ostringstream os;
  os << '{';
  os << "\"total_pairings\":" << pairings.size();
  int valid_pairings = 0;
  for (const auto& [key, pairing] : pairings) {
    if (pairing.valid) {
      ++valid_pairings;
    }
  }
  os << ",\"valid_pairings\":" << valid_pairings;
  os << ",\"sync_points_total\":" << sync_points_total;
  os << ",\"sync_points_accepted\":" << sync_points_accepted;
  os << ",\"groups_corrected\":" << groups_corrected;
  os << ",\"pair_details\":{";
  int emitted = 0;
  bool first = true;
  for (const auto& [key, pairing] : pairings) {
    if (!pairing.valid || emitted >= 10) {
      continue;
    }
    if (!first) {
      os << ',';
    }
    first = false;
    os << '"' << pairing.sensor_a << '-' << pairing.sensor_b << "\":{";
    os << "\"offset_us\":" << round_to(pairing.offset * 1e6, 1);
    os << ",\"drift_ppm\":" << round_to(pairing.drift * 1e6, 3);
    os << ",\"variance_us2\":" << round_to(pairing.variance * 1e12, 1);
    os << ",\"n\":" << pairing.n;
    os << '}';
    ++emitted;
  }
  os << "}}";
  return os.str();
}

void Layer4Stats::record_solve(const std::string& method, double residual_m) {
  ++groups_solved;
  ++method_counts[method];
  residuals.push_back(residual_m);
}

void Layer4Stats::record_sensor_residual(std::int64_t sensor_id, double residual_m) {
  auto& values = sensor_residuals_m[sensor_id];
  values.push_back(residual_m);
  if (values.size() > 100) {
    values.erase(values.begin());
  }
}

std::string Layer4Stats::to_json() const {
  std::ostringstream os;
  os << '{';
  os << "\"groups_received\":" << groups_received;
  os << ",\"groups_solved\":" << groups_solved;
  os << ",\"groups_failed\":" << groups_failed;
  os << ",\"groups_skipped_sensors\":" << groups_skipped_sensors;
  os << ",\"parse_errors\":" << parse_errors;
  double solve_rate = groups_received > 0 ? 100.0 * static_cast<double>(groups_solved) / static_cast<double>(groups_received) : 0.0;
  os << ",\"solve_rate\":\"" << round_to(solve_rate, 1) << "%\"";
  os << ",\"methods\":{";
  bool first = true;
  for (const auto& [name, count] : method_counts) {
    if (!first) os << ',';
    first = false;
    os << '"' << json_escape(name) << "\":" << count;
  }
  os << '}';
  os << ",\"failure_reasons\":{";
  first = true;
  for (const auto& [name, count] : failure_reasons) {
    if (!first) os << ',';
    first = false;
    os << '"' << json_escape(name) << "\":" << count;
  }
  os << '}';
  if (!residuals.empty()) {
    auto sorted = residuals;
    std::sort(sorted.begin(), sorted.end());
    std::size_t n = sorted.size();
    os << ",\"median_residual_m\":" << round_to(sorted[n / 2], 2);
    os << ",\"p95_residual_m\":" << round_to(sorted[n > 20 ? static_cast<std::size_t>(n * 0.95) : n - 1], 2);
  }
  os << ",\"clock_sync_bias_m\":{";
  first = true;
  for (const auto& [sensor_id, values] : sensor_residuals_m) {
    if (values.size() < 10) {
      continue;
    }
    if (!first) os << ',';
    first = false;
    double sum = 0.0;
    for (double v : values) sum += v;
    os << '"' << sensor_id << "\":" << round_to(sum / static_cast<double>(values.size()), 2);
  }
  os << '}';
  os << '}';
  return os.str();
}

}
