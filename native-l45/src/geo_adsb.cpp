// geo_adsb.cpp — Geodetic utilities and ADS-B message decoding.
// WGS84 coordinate conversions (LLA <-> ECEF), atmospheric refraction model,
// ADS-B DF17 position (CPR) and velocity extraction, and CPR global/local decode.
#include "core.hpp"

#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>

namespace native_l45 {

namespace {

// --- WGS84 ellipsoid parameters ---
constexpr double kWgs84A = 6378137.0;                    // semi-major axis (m)
constexpr double kWgs84F = 1.0 / 298.257223563;         // flattening
constexpr double kWgs84B = kWgs84A * (1.0 - kWgs84F);   // semi-minor axis (m)
constexpr double kWgs84E2 = 1.0 - (kWgs84B / kWgs84A) * (kWgs84B / kWgs84A);   // first eccentricity squared
constexpr double kWgs84Ep2 = (kWgs84A * kWgs84A - kWgs84B * kWgs84B) / (kWgs84B * kWgs84B); // second eccentricity squared

// --- Atmospheric refraction model constants ---
constexpr double kA0 = 315e-6;         // refractivity at sea level
constexpr double kB = 0.1361e-3;       // exponential decay rate (1/m)

// --- CPR decoding constants ---
constexpr double kCprPairWindow = 10.0; // max time between even/odd frames (s)
constexpr int kNz = 15;                 // number of latitude zones

// Cache key for sensor ECEF lookup (bit-exact match on LLA coordinates).
struct SensorEcefKey {
  std::uint64_t lat_bits = 0;
  std::uint64_t lon_bits = 0;
  std::uint64_t alt_bits = 0;

  bool operator==(const SensorEcefKey& other) const = default;
};

// Hash combining lat/lon/alt bit patterns (boost-style hash combine).
struct SensorEcefKeyHash {
  std::size_t operator()(const SensorEcefKey& key) const {
    std::size_t h = static_cast<std::size_t>(key.lat_bits);
    h ^= static_cast<std::size_t>(key.lon_bits) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    h ^= static_cast<std::size_t>(key.alt_bits) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
    return h;
  }
};

// Build a cache key from sensor coordinates by bit-casting doubles to uint64.
SensorEcefKey make_sensor_ecef_key(double lat_deg, double lon_deg, double alt_m) {
  return {
      std::bit_cast<std::uint64_t>(lat_deg),
      std::bit_cast<std::uint64_t>(lon_deg),
      std::bit_cast<std::uint64_t>(alt_m),
  };
}

// Convert a single hex character to its 4-bit value.
int hex_nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return 10 + c - 'a';
  if (c >= 'A' && c <= 'F') return 10 + c - 'A';
  return -1;
}

// Positive modulo (always returns [0, modulus)).
double positive_mod(double value, double modulus) {
  return value - modulus * std::floor(value / modulus);
}

// Parse a hex string into raw bytes.
bool hex_bytes(std::string_view hex, std::uint8_t* out, std::size_t count) {
  if (hex.size() != count * 2) {
    return false;
  }
  for (std::size_t i = 0; i < count; ++i) {
    int hi = hex_nibble(hex[i * 2]);
    int lo = hex_nibble(hex[i * 2 + 1]);
    if (hi < 0 || lo < 0) {
      return false;
    }
    out[i] = static_cast<std::uint8_t>((hi << 4) | lo);
  }
  return true;
}

// Decode Mode-S altitude code (Q-bit encoding) to feet.
std::optional<int> decode_altitude(int alt_code) {
  int q_bit = (alt_code >> 4) & 1;
  if (q_bit != 1) {
    return std::nullopt;
  }
  int n = ((alt_code & 0xFE0) >> 1) | (alt_code & 0x00F);
  int alt_ft = n * 25 - 1000;
  if (alt_ft < -1000 || alt_ft > 100000) {
    return std::nullopt;
  }
  return alt_ft;
}

// CPR NL function: number of longitude zones for a given latitude.
double cpr_nl(double lat) {
  if (std::abs(lat) >= 87.0) {
    return 1.0;
  }
  try {
    double cos_lat = std::cos(std::abs(lat) * M_PI / 180.0);
    double nl = std::floor(
        2.0 * M_PI /
        std::acos(1.0 - (1.0 - std::cos(M_PI / (2.0 * static_cast<double>(kNz)))) / (cos_lat * cos_lat)));
    return std::max(nl, 1.0);
  } catch (...) {
    return 1.0;
  }
}

// CPR global decode: requires both even and odd frames to compute unambiguous position.
std::optional<std::pair<double, double>> cpr_global_decode(
    int lat_cpr_even,
    int lon_cpr_even,
    int lat_cpr_odd,
    int lon_cpr_odd,
    bool most_recent_is_even) {
  double d_lat_even = 360.0 / (4.0 * kNz);
  double d_lat_odd = 360.0 / (4.0 * kNz - 1.0);
  double lat_even = static_cast<double>(lat_cpr_even) / 131072.0;
  double lon_even = static_cast<double>(lon_cpr_even) / 131072.0;
  double lat_odd = static_cast<double>(lat_cpr_odd) / 131072.0;
  double lon_odd = static_cast<double>(lon_cpr_odd) / 131072.0;
  double j = std::floor(59.0 * lat_even - 60.0 * lat_odd + 0.5);
  double lat_e = d_lat_even * (positive_mod(j, 60.0) + lat_even);
  double lat_o = d_lat_odd * (positive_mod(j, 59.0) + lat_odd);
  if (lat_e >= 270.0) {
    lat_e -= 360.0;
  }
  if (lat_o >= 270.0) {
    lat_o -= 360.0;
  }
  int nl_e = static_cast<int>(cpr_nl(lat_e));
  int nl_o = static_cast<int>(cpr_nl(lat_o));
  if (nl_e != nl_o) {
    return std::nullopt;
  }
  double lat = 0.0;
  int nl = 0;
  int ni = 0;
  if (most_recent_is_even) {
    lat = lat_e;
    nl = nl_e;
    ni = std::max(nl, 1);
  } else {
    lat = lat_o;
    nl = nl_o;
    ni = std::max(nl - 1, 1);
  }
  double m = std::floor(lon_even * static_cast<double>(nl - 1) - lon_odd * static_cast<double>(nl) + 0.5);
  double lon = 0.0;
  if (most_recent_is_even) {
    lon = (360.0 / static_cast<double>(ni)) * (positive_mod(m, static_cast<double>(ni)) + lon_even);
  } else {
    lon = (360.0 / static_cast<double>(ni)) * (positive_mod(m, static_cast<double>(ni)) + lon_odd);
  }
  if (lon >= 180.0) {
    lon -= 360.0;
  }
  if (lat < -90.0 || lat > 90.0) {
    return std::nullopt;
  }
  return std::make_pair(lat, lon);
}

// CPR local decode: uses a nearby reference position to resolve a single frame.
std::optional<std::pair<double, double>> cpr_local_decode(
    int lat_cpr,
    int lon_cpr,
    int f_bit,
    double ref_lat,
    double ref_lon) {
  double d_lat = f_bit == 0 ? 360.0 / (4.0 * kNz) : 360.0 / (4.0 * kNz - 1.0);
  double lat_cpr_norm = static_cast<double>(lat_cpr) / 131072.0;
  double j = std::floor(ref_lat / d_lat) + std::floor(0.5 + positive_mod(ref_lat, d_lat) / d_lat - lat_cpr_norm);
  double lat = d_lat * (j + lat_cpr_norm);
  if (std::abs(lat - ref_lat) > 5.0) {
    return std::nullopt;
  }
  int nl = static_cast<int>(cpr_nl(lat));
  int ni = f_bit == 0 ? std::max(nl, 1) : std::max(nl - 1, 1);
  double d_lon = 360.0 / static_cast<double>(ni);
  double lon_cpr_norm = static_cast<double>(lon_cpr) / 131072.0;
  double m = std::floor(ref_lon / d_lon) + std::floor(0.5 + positive_mod(ref_lon, d_lon) / d_lon - lon_cpr_norm);
  double lon = d_lon * (m + lon_cpr_norm);
  if (lon >= 180.0) {
    lon -= 360.0;
  }
  if (lon < -180.0) {
    lon += 360.0;
  }
  if (std::abs(lon - ref_lon) > 5.0) {
    return std::nullopt;
  }
  return std::make_pair(lat, lon);
}

}  // namespace

// Convert geodetic LLA (lat/lon in degrees, alt in metres) to ECEF XYZ.
Vec3 lla_to_ecef(double lat_deg, double lon_deg, double alt_m) {
  double lat = lat_deg * M_PI / 180.0;
  double lon = lon_deg * M_PI / 180.0;
  double sin_lat = std::sin(lat);
  double cos_lat = std::cos(lat);
  double sin_lon = std::sin(lon);
  double cos_lon = std::cos(lon);
  double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
  return {
      (n + alt_m) * cos_lat * cos_lon,
      (n + alt_m) * cos_lat * sin_lon,
      (n * (1.0 - kWgs84E2) + alt_m) * sin_lat,
  };
}

// Cached version of lla_to_ecef for sensor positions (avoids repeated trig).
Vec3 sensor_lla_to_ecef(double lat_deg, double lon_deg, double alt_m) {
  static std::unordered_map<SensorEcefKey, Vec3, SensorEcefKeyHash> cache = []() {
    std::unordered_map<SensorEcefKey, Vec3, SensorEcefKeyHash> map;
    map.reserve(64);
    return map;
  }();
  SensorEcefKey key = make_sensor_ecef_key(lat_deg, lon_deg, alt_m);
  auto it = cache.find(key);
  if (it != cache.end()) {
    return it->second;
  }
  Vec3 value = lla_to_ecef(lat_deg, lon_deg, alt_m);
  cache.emplace(key, value);
  return value;
}

// Convert ECEF XYZ to geodetic LLA using Bowring's iterative method.
std::tuple<double, double, double> ecef_to_lla(double x, double y, double z) {
  double lon = std::atan2(y, x);
  double p = std::hypot(x, y);
  double theta = std::atan2(z * kWgs84A, p * kWgs84B);
  double sin_theta = std::sin(theta);
  double cos_theta = std::cos(theta);
  double sin_theta3 = sin_theta * sin_theta * sin_theta;
  double cos_theta3 = cos_theta * cos_theta * cos_theta;
  double lat = std::atan2(z + kWgs84Ep2 * kWgs84B * sin_theta3, p - kWgs84E2 * kWgs84A * cos_theta3);
  double sin_lat = std::sin(lat);
  double n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
  lat = std::atan2(z + kWgs84E2 * n * sin_lat, p);
  sin_lat = std::sin(lat);
  double cos_lat = std::cos(lat);
  n = kWgs84A / std::sqrt(1.0 - kWgs84E2 * sin_lat * sin_lat);
  double alt = std::abs(cos_lat) > 1e-10 ? p / cos_lat - n : std::abs(z) / std::abs(sin_lat) - n * (1.0 - kWgs84E2);
  return {lat * 180.0 / M_PI, lon * 180.0 / M_PI, alt};
}

double ft_to_m(double feet) {
  return feet * 0.3048;
}

double m_to_ft(double meters) {
  return meters / 0.3048;
}

// Compute effective signal propagation velocity between sensor and aircraft altitudes,
// accounting for atmospheric refraction via exponential atmosphere model.
double effective_velocity(double h_sensor, double h_aircraft) {
  double dh = h_aircraft - h_sensor;
  if (std::abs(dh) < 1.0) {
    return kVacuumC / (1.0 + kA0 * std::exp(-kB * h_sensor));
  }
  double correction = kA0 / (kB * dh) * (std::exp(-kB * h_sensor) - std::exp(-kB * h_aircraft));
  return kVacuumC / (1.0 + correction);
}

// Extract CPR position fields (lat_cpr, lon_cpr, f_bit, altitude) from a DF17 hex message.
std::optional<CPRFrame> extract_df17_position_fields(const std::string& raw_msg) {
  if (raw_msg.size() != 28) {
    return std::nullopt;
  }
  int first_nibble_hi = hex_nibble(raw_msg[0]);
  int first_nibble_lo = hex_nibble(raw_msg[1]);
  if (first_nibble_hi < 0 || first_nibble_lo < 0) {
    return std::nullopt;
  }
  int first_byte = (first_nibble_hi << 4) | first_nibble_lo;
  int df = first_byte >> 3;
  if (df != 17) {
    return std::nullopt;
  }
  std::uint8_t me[7]{};
  if (!hex_bytes(std::string_view(raw_msg).substr(8, 14), me, 7)) {
    return std::nullopt;
  }
  int tc = me[0] >> 3;
  if (tc < 9 || tc > 18) {
    return std::nullopt;
  }
  int alt_code = ((me[0] & 0x07) << 9) | (me[1] << 1) | (me[2] >> 7);
  CPRFrame frame;
  frame.f_bit = (me[2] >> 2) & 1;
  frame.lat_cpr = ((me[2] & 0x03) << 15) | (me[3] << 7) | (me[4] >> 1);
  frame.lon_cpr = ((me[4] & 0x01) << 16) | (me[5] << 8) | me[6];
  frame.alt_ft = decode_altitude(alt_code);
  frame.timestamp = 0.0;
  return frame;
}

// Extract airborne velocity (ground speed, heading, vertical rate) from a DF17 hex message.
std::optional<DecodedVelocity> extract_df17_velocity(const std::string& raw_msg) {
  if (raw_msg.size() != 28) {
    return std::nullopt;
  }
  int first_nibble_hi = hex_nibble(raw_msg[0]);
  int first_nibble_lo = hex_nibble(raw_msg[1]);
  if (first_nibble_hi < 0 || first_nibble_lo < 0) {
    return std::nullopt;
  }
  int first_byte = (first_nibble_hi << 4) | first_nibble_lo;
  int df = first_byte >> 3;
  if (df != 17) {
    return std::nullopt;
  }
  std::uint8_t me[7]{};
  if (!hex_bytes(std::string_view(raw_msg).substr(8, 14), me, 7)) {
    return std::nullopt;
  }
  int tc = me[0] >> 3;
  if (tc != 19) {
    return std::nullopt;
  }
  int st = me[0] & 0x07;
  if (st != 1) {
    return std::nullopt;
  }
  int ew_dir = (me[1] >> 2) & 1;
  int ew_vel = ((me[1] & 0x03) << 8) | me[2];
  int ns_dir = (me[3] >> 7) & 1;
  int ns_vel = ((me[3] & 0x7F) << 3) | (me[4] >> 5);
  if (ew_vel == 0 || ns_vel == 0) {
    return std::nullopt;
  }
  double ew_knots = static_cast<double>(ew_vel - 1);
  double ns_knots = static_cast<double>(ns_vel - 1);
  if (ew_dir) ew_knots = -ew_knots;
  if (ns_dir) ns_knots = -ns_knots;
  double speed = std::sqrt(ew_knots * ew_knots + ns_knots * ns_knots);
  double heading = std::fmod(std::atan2(ew_knots, ns_knots) * 180.0 / M_PI + 360.0, 360.0);
  int vr_sign = (me[4] >> 4) & 1;
  int vr_raw = ((me[4] & 0x0F) << 5) | (me[5] >> 3);
  int vrate_fpm = 0;
  if (vr_raw != 0) {
    vrate_fpm = (vr_raw - 1) * 64;
    if (vr_sign) {
      vrate_fpm = -vrate_fpm;
    }
  }
  return DecodedVelocity{ew_knots, ns_knots, speed, heading, vrate_fpm};
}

// Convert a decoded ADS-B position to ECEF (default 10 km altitude if missing).
Vec3 position_to_ecef(double lat, double lon, const std::optional<int>& alt_ft) {
  double alt_m = alt_ft ? ft_to_m(static_cast<double>(*alt_ft)) : 10000.0;
  return lla_to_ecef(lat, lon, alt_m);
}

// Add a CPR frame for an ICAO: try global decode first, fall back to local decode.
std::optional<ADSBPosition> CPRBuffer::add_frame(const std::string& icao, const CPRFrame& frame) {
  ++frames_received;
  auto& pair = frames_[icao];
  if (frame.f_bit == 0) {
    pair.even = frame;
  } else {
    pair.odd = frame;
  }
  if (pair.even && pair.odd) {
    double dt = std::abs(pair.even->timestamp - pair.odd->timestamp);
    if (dt < kCprPairWindow) {
      bool most_recent_is_even = pair.even->timestamp >= pair.odd->timestamp;
      auto decoded = cpr_global_decode(
          pair.even->lat_cpr,
          pair.even->lon_cpr,
          pair.odd->lat_cpr,
          pair.odd->lon_cpr,
          most_recent_is_even);
      if (decoded) {
        references_[icao] = *decoded;
        ++global_decodes;
        return ADSBPosition{decoded->first, decoded->second, frame.alt_ft, frame.timestamp, icao};
      }
    }
  }
  auto reference_it = references_.find(icao);
  if (reference_it != references_.end()) {
    auto decoded = cpr_local_decode(frame.lat_cpr, frame.lon_cpr, frame.f_bit, reference_it->second.first, reference_it->second.second);
    if (decoded) {
      reference_it->second = *decoded;
      ++local_decodes;
      return ADSBPosition{decoded->first, decoded->second, frame.alt_ft, frame.timestamp, icao};
    }
  }
  return std::nullopt;
}

std::unordered_map<std::string, double> CPRBuffer::stats_dict() const {
  return {
      {"frames_received", static_cast<double>(frames_received)},
      {"global_decodes", static_cast<double>(global_decodes)},
      {"local_decodes", static_cast<double>(local_decodes)},
      {"tracked_icaos", static_cast<double>(frames_.size())},
  };
}

std::size_t CPRBuffer::tracked_icaos() const {
  return frames_.size();
}

}
