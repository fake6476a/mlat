#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <iosfwd>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace native_l45 {

struct Vec3 {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;

  Vec3& operator+=(const Vec3& other) {
    x += other.x;
    y += other.y;
    z += other.z;
    return *this;
  }

  Vec3& operator-=(const Vec3& other) {
    x -= other.x;
    y -= other.y;
    z -= other.z;
    return *this;
  }

  Vec3& operator*=(double scalar) {
    x *= scalar;
    y *= scalar;
    z *= scalar;
    return *this;
  }

  Vec3& operator/=(double scalar) {
    x /= scalar;
    y /= scalar;
    z /= scalar;
    return *this;
  }
};

inline Vec3 operator+(Vec3 lhs, const Vec3& rhs) {
  return lhs += rhs;
}

inline Vec3 operator-(Vec3 lhs, const Vec3& rhs) {
  return lhs -= rhs;
}

inline Vec3 operator*(Vec3 lhs, double scalar) {
  return lhs *= scalar;
}

inline Vec3 operator*(double scalar, Vec3 rhs) {
  return rhs *= scalar;
}

inline Vec3 operator/(Vec3 lhs, double scalar) {
  return lhs /= scalar;
}

inline double dot(const Vec3& a, const Vec3& b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline double norm_sq(const Vec3& v) {
  return dot(v, v);
}

inline double norm(const Vec3& v) {
  return std::sqrt(norm_sq(v));
}

struct Mat3 {
  std::array<std::array<double, 3>, 3> m{};
};

struct Mat6 {
  std::array<std::array<double, 6>, 6> m{};
};

struct Reception {
  std::int64_t sensor_id = 0;
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
  Vec3 sensor_ecef{};
  std::int64_t timestamp_s = 0;
  std::int64_t timestamp_ns = 0;
};

struct Group {
  std::string icao;
  int df_type = 0;
  std::optional<double> altitude_ft;
  std::optional<std::string> squawk;
  std::string raw_msg;
  int num_sensors = 0;
  std::vector<Reception> receptions;
};

struct SolveFix {
  std::string icao;
  double lat = 0.0;
  double lon = 0.0;
  double alt_ft = 0.0;
  double residual_m = 0.0;
  double quality_residual_m = 0.0;
  double gdop = 0.0;
  int num_sensors = 0;
  std::string solve_method;
  std::int64_t timestamp_s = 0;
  std::int64_t timestamp_ns = 0;
  int df_type = 0;
  std::optional<std::string> squawk;
  std::string raw_msg;
  double t0_s = 0.0;
};

struct Layer4Record {
  bool is_unsolved_group = false;
  Group unsolved_group;
  SolveFix fix;
};

struct TrackOutput {
  std::string icao;
  double lat = 0.0;
  double lon = 0.0;
  double alt_ft = 0.0;
  double heading_deg = 0.0;
  double speed_kts = 0.0;
  double vrate_fpm = 0.0;
  int track_quality = 0;
  int positions_count = 0;
  double residual_m = 0.0;
  double quality_residual_m = 0.0;
  double gdop = 0.0;
  int num_sensors = 0;
  std::string solve_method;
  std::int64_t timestamp_s = 0;
  std::int64_t timestamp_ns = 0;
  int df_type = 0;
  std::optional<std::string> squawk;
  std::string raw_msg;
  double t0_s = 0.0;
  std::array<std::array<double, 2>, 2> cov_matrix{};
};

struct JsonValue {
  using Array = std::vector<JsonValue>;
  using Object = std::vector<std::pair<std::string, JsonValue>>;
  using Storage = std::variant<std::nullptr_t, bool, double, std::string, Array, Object>;

  Storage storage;

  bool is_null() const;
  bool is_bool() const;
  bool is_number() const;
  bool is_string() const;
  bool is_array() const;
  bool is_object() const;
  bool as_bool(bool fallback = false) const;
  double as_number(double fallback = 0.0) const;
  const std::string& as_string() const;
  const Array& as_array() const;
  const Object& as_object() const;
};

class JsonParser {
 public:
  explicit JsonParser(std::string_view input);
  JsonValue parse();

 private:
  std::string_view input_;
  std::size_t pos_ = 0;

  void skip_ws();
  char peek() const;
  char get();
  bool consume(char c);
  void expect(char c);
  JsonValue parse_value();
  JsonValue parse_null();
  JsonValue parse_true();
  JsonValue parse_false();
  JsonValue parse_number();
  JsonValue parse_string();
  JsonValue parse_array();
  JsonValue parse_object();
  static int hex_value(char c);
};

const JsonValue* json_find(const JsonValue::Object& object, std::string_view key);
std::optional<std::string> json_get_string_optional(const JsonValue::Object& object, std::string_view key);
std::optional<double> json_get_number_optional(const JsonValue::Object& object, std::string_view key);
std::optional<std::int64_t> json_get_int_optional(const JsonValue::Object& object, std::string_view key);
std::optional<bool> json_get_bool_optional(const JsonValue::Object& object, std::string_view key);

bool parse_group_json(std::string_view line, Group& out);
bool parse_layer4_record_json(std::string_view line, Layer4Record& out);
std::string to_json_unsolved_group(const Group& group);
std::string to_json_fix(const SolveFix& fix);
std::string to_json_track(const TrackOutput& track);

std::string json_escape(std::string_view value);
double round_to(double value, int digits);
double clamp(double value, double lo, double hi);

constexpr double kVacuumC = 299792458.0;
constexpr double kAirC = 299702547.0;

Vec3 lla_to_ecef(double lat_deg, double lon_deg, double alt_m);
Vec3 sensor_lla_to_ecef(double lat_deg, double lon_deg, double alt_m);
std::tuple<double, double, double> ecef_to_lla(double x, double y, double z);
double ft_to_m(double feet);
double m_to_ft(double meters);
double effective_velocity(double h_sensor, double h_aircraft);

struct CPRFrame {
  int f_bit = 0;
  int lat_cpr = 0;
  int lon_cpr = 0;
  std::optional<int> alt_ft;
  double timestamp = 0.0;
};

struct DecodedVelocity {
  double ew_knots = 0.0;
  double ns_knots = 0.0;
  double speed_knots = 0.0;
  double heading_deg = 0.0;
  int vrate_fpm = 0;
};

struct ADSBPosition {
  double lat = 0.0;
  double lon = 0.0;
  std::optional<int> alt_ft;
  double timestamp = 0.0;
  std::string icao;
};

class CPRBuffer {
 public:
  std::optional<ADSBPosition> add_frame(const std::string& icao, const CPRFrame& frame);
  std::unordered_map<std::string, double> stats_dict() const;
  std::size_t tracked_icaos() const;
  int global_decodes = 0;
  int local_decodes = 0;
  int frames_received = 0;

 private:
  struct FramePair {
    std::optional<CPRFrame> even;
    std::optional<CPRFrame> odd;
  };

  std::unordered_map<std::string, FramePair> frames_;
  std::unordered_map<std::string, std::pair<double, double>> references_;
};

std::optional<CPRFrame> extract_df17_position_fields(const std::string& raw_msg);
std::optional<DecodedVelocity> extract_df17_velocity(const std::string& raw_msg);
Vec3 position_to_ecef(double lat, double lon, const std::optional<int>& alt_ft);

struct OverrideEntry {
  std::string public_key;
  double lat = 0.0;
  double lon = 0.0;
  double alt = 0.0;
  std::string name;
};

class SensorOverrideMap {
 public:
  explicit SensorOverrideMap(std::vector<OverrideEntry> overrides = {});
  const OverrideEntry* lookup(std::int64_t sensor_id, double stream_lat, double stream_lon);
  bool empty() const;
  std::string stats_json() const;

 private:
  std::vector<OverrideEntry> overrides_;
  std::unordered_map<std::int64_t, std::size_t> sensor_map_;
  std::unordered_map<std::int64_t, bool> rejected_;
  int matched_count_ = 0;
};

std::optional<SensorOverrideMap> load_location_overrides();
std::vector<Reception> apply_overrides(const std::vector<Reception>& receptions, SensorOverrideMap& override_map);

struct CachedPosition {
  Vec3 ecef;
  double lat = 0.0;
  double lon = 0.0;
  double alt_m = 0.0;
  double timestamp = 0.0;
  std::optional<Vec3> velocity_ecef;
  double residual_m = 0.0;
  int solve_count = 1;
  double last_order = 0.0;

  Vec3 predict(double target_timestamp) const;
  bool is_physically_consistent(const Vec3& new_ecef, double new_timestamp, double new_residual_m = 0.0) const;
};

class PositionCache {
 public:
  std::optional<CachedPosition> get(const std::string& icao, std::optional<double> target_timestamp = std::nullopt);
  void put(const std::string& icao, const Vec3& ecef, double lat, double lon, double alt_m, double timestamp, double residual_m = 0.0);
  void update_velocity_from_adsb(const std::string& icao, double ew_mps, double ns_mps, double vrate_mps, double timestamp);
  std::string stats_json() const;
  std::size_t size() const;

 private:
  void prune();
  std::unordered_map<std::string, CachedPosition> cache_;
  mutable std::uint64_t order_counter_ = 0;
  std::uint64_t hits_ = 0;
  std::uint64_t misses_ = 0;
};

class ClockPairing {
 public:
  ClockPairing() = default;
  ClockPairing(std::int64_t sensor_a, std::int64_t sensor_b);

  bool update(double measured_offset, double now);
  double predict(double now) const;

  std::int64_t sensor_a = 0;
  std::int64_t sensor_b = 0;
  int n = 0;
  double offset = 0.0;
  double drift = 0.0;
  double variance = 2.5e-9;
  double p00 = 2.5e-9;
  double p01 = 0.0;
  double p11 = 1e-12;
  bool valid = false;
  double last_update = 0.0;
  int consecutive_outliers = 0;
};

class ClockCalibrator {
 public:
  void process_adsb_reference(const Vec3& aircraft_ecef, const std::vector<Reception>& receptions, double now);
  std::vector<Reception> correct_timestamps(const std::vector<Reception>& receptions, double now);
  bool has_any_calibration(const std::vector<Reception>& receptions) const;
  std::string stats_json() const;
  static std::uint64_t pair_key(std::int64_t a, std::int64_t b);
  std::unordered_map<std::uint64_t, ClockPairing> pairings;
  int sync_points_total = 0;
  int sync_points_accepted = 0;
  int groups_corrected = 0;
  int calibrated_pairs = 0;

 private:
  ClockPairing& get_pairing(std::int64_t a, std::int64_t b);
};

double compute_gdop(const Vec3& aircraft_pos, const std::vector<Vec3>& sensor_positions);
double compute_gdop_2d(const Vec3& aircraft_pos, const std::vector<Vec3>& sensor_positions);

struct ToaResult {
  Vec3 position;
  double residual_m = 0.0;
  double objective_residual_m = 0.0;
  double t0_s = 0.0;
  double cost = 0.0;
  int nfev = 0;
  bool success = false;
};

std::optional<Vec3> inamdar_5sensor(const std::vector<Vec3>& sensors, const std::vector<double>& arrival_times);
std::optional<Vec3> inamdar_4sensor_altitude(const std::vector<Vec3>& sensors, const std::vector<double>& arrival_times, double altitude_m);
Vec3 centroid_init(const std::vector<Vec3>& sensors, std::optional<double> altitude_m);
std::optional<ToaResult> solve_toa(
    const std::vector<Vec3>& sensors,
    const std::vector<double>& arrival_times,
    const std::vector<double>& sensor_alts_m,
    const Vec3& x0,
    std::optional<double> altitude_m = std::nullopt,
    const std::optional<Vec3>& track_prediction_ecef = std::nullopt,
    int max_nfev = 1000,
    double prediction_weight = 8.0);
std::optional<ToaResult> solve_constrained_3sensor(
    const std::vector<Vec3>& sensors,
    const std::vector<double>& arrival_times,
    const std::vector<double>& sensor_alts_m,
    double altitude_m,
    const Vec3& x0);

struct SolveOutcome {
  std::optional<SolveFix> result;
  std::string fail_reason;
};

SolveOutcome solve_group(const Group& group, const std::optional<Vec3>& position_prior_ecef);

class Layer4Stats {
 public:
  void record_solve(const std::string& method, double residual_m);
  void record_sensor_residual(std::int64_t sensor_id, double residual_m);
  std::string to_json() const;

  int groups_received = 0;
  int groups_solved = 0;
  int groups_failed = 0;
  int groups_skipped_sensors = 0;
  int parse_errors = 0;
  std::unordered_map<std::string, int> method_counts;
  std::unordered_map<std::string, int> failure_reasons;
  std::vector<double> residuals;
  std::unordered_map<std::int64_t, std::vector<double>> sensor_residuals_m;
};

class Layer4Processor {
 public:
  Layer4Processor();
  bool process_line(const std::string& line, std::ostream& out);
  void finish(std::ostream& log) const;

 private:
  void process_group(Group group, std::ostream& out);

  Layer4Stats stats_;
  ClockCalibrator clock_cal_;
  PositionCache pos_cache_;
  CPRBuffer cpr_buffer_;
  std::optional<SensorOverrideMap> override_map_;
  int cpr_seeds_ = 0;
  int df17_alt_extracted_ = 0;
  int cached_alt_used_ = 0;
};

int run_layer4(std::istream& in, std::ostream& out, std::ostream& log);

class AircraftEKF {
 public:
  AircraftEKF(const Vec3& position, double timestamp_s, double process_accel = 5.0, double meas_noise = 200.0);
  Vec3 predict(double timestamp_s);
  double update(const Vec3& measurement, double timestamp_s, std::optional<double> measurement_noise_m = std::nullopt);
  Vec3 position() const;
  Vec3 velocity() const;
  double median_innovation() const;

  Mat6 P{};
  std::array<double, 6> x{};
  double last_timestamp_s = 0.0;
  int updates = 1;
  std::vector<double> innovations;
  double process_accel = 5.0;
  double meas_noise = 200.0;
};

struct TrackState {
  std::string icao;
  AircraftEKF ekf;
  std::vector<std::array<double, 4>> positions;
  std::vector<double> timestamps;
  double last_update_order = 0.0;
  double creation_order = 0.0;
  double last_alt_ft = 0.0;
  int last_df_type = 0;
  std::optional<std::string> last_squawk;

  TrackState(const std::string& icao_, const Vec3& position_ecef, double timestamp_s, double alt_ft, int df_type, std::optional<std::string> squawk, double order);
  double update(const Vec3& position_ecef, double timestamp_s, double alt_ft, int df_type, std::optional<std::string> squawk, std::optional<double> measurement_noise_m, double order);
  double age(double now_order) const;
  bool is_established() const;
  double heading_deg() const;
  double ground_speed_kts() const;
  double vertical_rate_fpm() const;
  TrackOutput to_output(const SolveFix& fix) const;
};

class TrackManager {
 public:
  std::optional<TrackOutput> process_record(const Layer4Record& record);
  int prune_stale();
  std::string stats_json() const;

 private:
  std::optional<TrackOutput> process_fix(const SolveFix& fix);
  std::optional<TrackOutput> solve_prediction_aided(const Group& group);
  std::unordered_map<std::string, TrackState> tracks_;
  mutable double order_counter_ = 0.0;

 public:
  int fixes_received = 0;
  int fixes_accepted = 0;
  int fixes_rejected = 0;
  int tracks_created = 0;
  int tracks_pruned = 0;
};

int run_layer5(std::istream& in, std::ostream& out, std::ostream& log);

}
