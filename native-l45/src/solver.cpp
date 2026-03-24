// solver.cpp — Layer 4 MLAT position solver.
// Solves aircraft positions from Time-of-Arrival (TOA) measurements using
// Levenberg-Marquardt optimization with Soft-L1 robust cost. Supports
// algebraic initialization (Inamdar), outlier rejection, altitude constraints,
// prior-aided 2-sensor solving, GDOP filtering, and clock calibration.
#include "core.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <numeric>
#include <tuple>

namespace native_l45 {

namespace {

// --- Quality thresholds ---
constexpr double kMaxRangeM = 500000.0;
constexpr double kMaxGdop = 20.0;                   // 3D GDOP cap (>=4 sensors)
constexpr double kMaxGdop2d = 10.0;                  // 2D GDOP cap (2-3 sensors)
constexpr double kMaxResidualM = 10000.0;            // general residual cap
constexpr double kMaxResidualPriorM = 200.0;         // residual cap for prior-aided solves
constexpr double kMaxPriorDriftM = 30000.0;          // max drift from prior (3+ sensors)
constexpr double kMaxPriorDrift2SensorM = 5000.0;    // max drift from prior (2 sensors)
constexpr int kMinSensorsNoAlt = 4;                  // min sensors without altitude
constexpr int kMinSensorsWithAlt = 3;                // min sensors with barometric alt
constexpr int kMinSensorsWithPrior = 2;              // min sensors with position prior
constexpr double kOutlierSensorThresholdM = 3000.0;  // per-sensor outlier rejection threshold

// Gaussian elimination with partial pivoting to solve Ax = b.
template <std::size_t N>
bool solve_linear(std::array<std::array<double, N>, N> a, std::array<double, N> b, std::array<double, N>& x) {
  for (std::size_t col = 0; col < N; ++col) {
    std::size_t pivot = col;
    double best = std::abs(a[col][col]);
    for (std::size_t row = col + 1; row < N; ++row) {
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
      std::swap(b[pivot], b[col]);
    }
    for (std::size_t row = col + 1; row < N; ++row) {
      double factor = a[row][col] / a[col][col];
      if (factor == 0.0) {
        continue;
      }
      for (std::size_t k = col; k < N; ++k) {
        a[row][k] -= factor * a[col][k];
      }
      b[row] -= factor * b[col];
    }
  }
  for (std::size_t i = N; i-- > 0;) {
    double sum = b[i];
    for (std::size_t j = i + 1; j < N; ++j) {
      sum -= a[i][j] * x[j];
    }
    if (std::abs(a[i][i]) < 1e-15) {
      return false;
    }
    x[i] = sum / a[i][i];
  }
  return true;
}

// Gauss-Jordan matrix inversion with partial pivoting.
template <std::size_t N>
bool invert_matrix(std::array<std::array<double, N>, N> a, std::array<std::array<double, N>, N>& inv) {
  for (std::size_t i = 0; i < N; ++i) {
    for (std::size_t j = 0; j < N; ++j) {
      inv[i][j] = (i == j) ? 1.0 : 0.0;
    }
  }
  for (std::size_t col = 0; col < N; ++col) {
    std::size_t pivot = col;
    double best = std::abs(a[col][col]);
    for (std::size_t row = col + 1; row < N; ++row) {
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
    for (std::size_t j = 0; j < N; ++j) {
      a[col][j] /= div;
      inv[col][j] /= div;
    }
    for (std::size_t row = 0; row < N; ++row) {
      if (row == col) {
        continue;
      }
      double factor = a[row][col];
      if (factor == 0.0) {
        continue;
      }
      for (std::size_t j = 0; j < N; ++j) {
        a[row][j] -= factor * a[col][j];
        inv[row][j] -= factor * inv[col][j];
      }
    }
  }
  return true;
}

// Compute effective signal velocity per sensor, accounting for atmospheric refraction.
std::vector<double> compute_velocities(const std::vector<double>& sensor_alts_m, const std::optional<double>& aircraft_alt_m) {
  std::vector<double> velocities(sensor_alts_m.size(), kVacuumC);
  if (!aircraft_alt_m) {
    return velocities;
  }
  for (std::size_t i = 0; i < sensor_alts_m.size(); ++i) {
    velocities[i] = effective_velocity(sensor_alts_m[i], *aircraft_alt_m);
  }
  return velocities;
}

// Intermediate state for TOA residual computation.
struct TimingState {
  std::vector<double> residuals;   // per-sensor timing residuals (metres)
  std::vector<double> ranges;      // distance from candidate to each sensor
  std::vector<Vec3> units;         // unit vectors from sensor to candidate
  double t0_hat = 0.0;             // estimated emission time (Frisch elimination)

  void resize(std::size_t n) {
    residuals.resize(n);
    ranges.resize(n);
    units.resize(n);
  }
};

// Compute timing residuals using Frisch-style t0 elimination.
// For each sensor: residual = (arrival_time - t0_hat) * velocity - range.
void timing_state_core(
    const Vec3& x,
    const std::vector<Vec3>& sensors,
    const std::vector<double>& arrival_times,
    const std::vector<double>& velocities,
    TimingState& state) {
  std::size_t n = sensors.size();
  state.resize(n);
  double t0_sum = 0.0;
  for (std::size_t i = 0; i < n; ++i) {
    Vec3 diff = x - sensors[i];
    double r = norm(diff);
    if (r < 1e-12) {
      state.units[i] = Vec3{};
      r = 1e-12;
    } else {
      state.units[i] = diff / r;
    }
    state.ranges[i] = r;
    t0_sum += arrival_times[i] - r / velocities[i];
  }
  state.t0_hat = t0_sum / static_cast<double>(n);
  for (std::size_t i = 0; i < n; ++i) {
    state.residuals[i] = (arrival_times[i] - state.t0_hat) * velocities[i] - state.ranges[i];
  }
}

// Jacobian of timing residuals w.r.t. position (3 columns per sensor row).
void timing_jacobian_core(
    const std::vector<Vec3>& units,
    const std::vector<double>& velocities,
    std::vector<std::array<double, 3>>& jac) {
  jac.resize(units.size());
  Vec3 sum_u_over_v{};
  for (std::size_t i = 0; i < units.size(); ++i) {
    double inv_v = 1.0 / velocities[i];
    sum_u_over_v += units[i] * inv_v;
  }
  double inv_n = 1.0 / static_cast<double>(units.size());
  for (std::size_t i = 0; i < units.size(); ++i) {
    double scale = velocities[i] * inv_n;
    jac[i] = {scale * sum_u_over_v.x - units[i].x, scale * sum_u_over_v.y - units[i].y, scale * sum_u_over_v.z - units[i].z};
  }
}

// Altitude constraint: penalises deviation from barometric altitude.
struct AltitudeConstraintEval {
  double residual = 0.0;
  std::array<double, 3> gradient{};
};

// Evaluate altitude constraint residual (and optionally gradient).
AltitudeConstraintEval evaluate_altitude_constraint(const Vec3& x, double aircraft_alt_m, bool with_gradient) {
  auto [lat, lon, alt] = ecef_to_lla(x.x, x.y, x.z);
  AltitudeConstraintEval eval;
  eval.residual = (alt - aircraft_alt_m) * 10.0;
  if (!with_gradient) {
    return eval;
  }
  double lat_r = lat * M_PI / 180.0;
  double lon_r = lon * M_PI / 180.0;
  double cos_lat = std::cos(lat_r);
  eval.gradient = {
      10.0 * cos_lat * std::cos(lon_r),
      10.0 * cos_lat * std::sin(lon_r),
      10.0 * std::sin(lat_r),
  };
  return eval;
}

// Full objective evaluation: timing residuals + altitude constraint + prediction prior.
struct ObjectiveEval {
  std::vector<double> objective_residuals;          // all residual terms
  std::vector<std::array<double, 3>> jacobian;     // Jacobian rows
  std::vector<double> timing_residuals;            // timing-only residuals
  double t0_hat = 0.0;

  void reserve(std::size_t sensor_count, bool has_altitude, bool has_prediction) {
    std::size_t extra = (has_altitude ? 1u : 0u) + (has_prediction ? 3u : 0u);
    objective_residuals.reserve(sensor_count + extra);
    jacobian.reserve(sensor_count + extra);
    timing_residuals.reserve(sensor_count);
  }
};

// Build the full objective: timing residuals + optional altitude + optional prediction prior.
void evaluate_objective(
    const Vec3& x,
    const std::vector<Vec3>& sensors,
    const std::vector<double>& arrival_times,
    const std::vector<double>& velocities,
    const std::optional<double>& aircraft_alt_m,
    const std::optional<Vec3>& track_prediction_ecef,
    double prediction_weight,
    TimingState& timing,
    ObjectiveEval& eval,
    bool with_jacobian) {
  timing_state_core(x, sensors, arrival_times, velocities, timing);
  eval.timing_residuals.assign(timing.residuals.begin(), timing.residuals.end());
  eval.t0_hat = timing.t0_hat;
  eval.objective_residuals.assign(timing.residuals.begin(), timing.residuals.end());
  std::optional<AltitudeConstraintEval> altitude_eval;
  if (aircraft_alt_m) {
    altitude_eval = evaluate_altitude_constraint(x, *aircraft_alt_m, with_jacobian);
    eval.objective_residuals.push_back(altitude_eval->residual);
  }
  if (track_prediction_ecef) {
    eval.objective_residuals.push_back((x.x - track_prediction_ecef->x) * prediction_weight);
    eval.objective_residuals.push_back((x.y - track_prediction_ecef->y) * prediction_weight);
    eval.objective_residuals.push_back((x.z - track_prediction_ecef->z) * prediction_weight);
  }
  if (!with_jacobian) {
    eval.jacobian.clear();
    return;
  }
  timing_jacobian_core(timing.units, velocities, eval.jacobian);
  if (aircraft_alt_m) {
    eval.jacobian.push_back(altitude_eval->gradient);
  }
  if (track_prediction_ecef) {
    eval.jacobian.push_back({prediction_weight, 0.0, 0.0});
    eval.jacobian.push_back({0.0, prediction_weight, 0.0});
    eval.jacobian.push_back({0.0, 0.0, prediction_weight});
  }
}

// Soft-L1 (pseudo-Huber) cost: robust to outlier residuals.
double soft_l1_cost(const std::vector<double>& residuals, double f_scale = 500.0) {
  double total = 0.0;
  for (double r : residuals) {
    double z = (r / f_scale) * (r / f_scale);
    total += 2.0 * f_scale * f_scale * (std::sqrt(1.0 + z) - 1.0);
  }
  return total;
}

// Per-row weight for IRLS (iteratively reweighted least squares) under Soft-L1.
double soft_l1_row_scale(double residual, double f_scale = 500.0) {
  double z = (residual / f_scale) * (residual / f_scale);
  return std::sqrt(1.0 / std::sqrt(1.0 + z));
}

// Core Levenberg-Marquardt TOA solver.
// Minimises Soft-L1 cost over position (3 unknowns), with t0 analytically eliminated.
std::optional<ToaResult> solve_toa_impl(
    const std::vector<Vec3>& sensors,
    const std::vector<double>& arrival_times,
    const std::vector<double>& sensor_alts_m,
    const Vec3& x0,
    const std::optional<double>& altitude_m,
    const std::optional<Vec3>& track_prediction_ecef,
    int max_nfev,
    double prediction_weight) {
  if (sensors.size() < 2 || sensors.size() != arrival_times.size() || sensors.size() != sensor_alts_m.size()) {
    return std::nullopt;
  }
  Vec3 x = x0;
  auto velocities = compute_velocities(sensor_alts_m, altitude_m);
  TimingState timing;
  TimingState candidate_timing;
  ObjectiveEval eval;
  ObjectiveEval candidate_eval;
  bool has_altitude = altitude_m.has_value();
  bool has_prediction = track_prediction_ecef.has_value();
  eval.reserve(sensors.size(), has_altitude, has_prediction);
  candidate_eval.reserve(sensors.size(), has_altitude, has_prediction);
  evaluate_objective(x, sensors, arrival_times, velocities, altitude_m, track_prediction_ecef, prediction_weight, timing, eval, true);
  double best_cost = soft_l1_cost(eval.objective_residuals);
  if (!std::isfinite(best_cost)) {
    return std::nullopt;
  }
  double lambda = 1e-3;
  int nfev = 1;
  for (int iter = 0; iter < max_nfev; ++iter) {
    std::array<std::array<double, 3>, 3> a{};
    std::array<double, 3> b{};
    for (std::size_t i = 0; i < eval.objective_residuals.size(); ++i) {
      double scale = soft_l1_row_scale(eval.objective_residuals[i]);
      double rw = eval.objective_residuals[i] * scale;
      std::array<double, 3> jw = {
          eval.jacobian[i][0] * scale,
          eval.jacobian[i][1] * scale,
          eval.jacobian[i][2] * scale,
      };
      for (int r = 0; r < 3; ++r) {
        b[r] -= jw[r] * rw;
        for (int c = 0; c < 3; ++c) {
          a[r][c] += jw[r] * jw[c];
        }
      }
    }
    for (int i = 0; i < 3; ++i) {
      a[i][i] += lambda;
    }
    std::array<double, 3> dx{};
    if (!solve_linear<3>(a, b, dx)) {
      return std::nullopt;
    }
    Vec3 candidate = x + Vec3{dx[0], dx[1], dx[2]};
    evaluate_objective(candidate, sensors, arrival_times, velocities, altitude_m, track_prediction_ecef, prediction_weight, candidate_timing, candidate_eval, true);
    ++nfev;
    double candidate_cost = soft_l1_cost(candidate_eval.objective_residuals);
    if (!std::isfinite(candidate_cost)) {
      lambda = std::min(lambda * 10.0, 1e12);
      continue;
    }
    if (candidate_cost + 1e-9 < best_cost) {
      x = candidate;
      std::swap(eval, candidate_eval);
      std::swap(timing, candidate_timing);
      best_cost = candidate_cost;
      lambda = std::max(lambda * 0.5, 1e-12);
      double step_norm = std::sqrt(dx[0] * dx[0] + dx[1] * dx[1] + dx[2] * dx[2]);
      if (step_norm < 1e-4) {
        break;
      }
    } else {
      lambda = std::min(lambda * 10.0, 1e12);
    }
  }
  Vec3 position = x;
  if (altitude_m) {
    auto [lat, lon, alt] = ecef_to_lla(position.x, position.y, position.z);
    static_cast<void>(alt);
    position = lla_to_ecef(lat, lon, *altitude_m);
  }
  evaluate_objective(position, sensors, arrival_times, velocities, altitude_m, track_prediction_ecef, prediction_weight, candidate_timing, candidate_eval, false);
  double residual_sum = 0.0;
  for (double r : candidate_eval.timing_residuals) {
    residual_sum += r * r;
  }
  double objective_sum = 0.0;
  for (double r : candidate_eval.objective_residuals) {
    objective_sum += r * r;
  }
  ToaResult result;
  result.position = position;
  result.residual_m = std::sqrt(residual_sum / static_cast<double>(candidate_eval.timing_residuals.size()));
  result.objective_residual_m = std::sqrt(objective_sum / static_cast<double>(candidate_eval.objective_residuals.size()));
  result.t0_s = candidate_eval.t0_hat;
  result.cost = best_cost;
  result.nfev = nfev;
  result.success = true;
  return result;
}

// Evaluate per-sensor timing residuals without Jacobian (used for outlier detection).
std::vector<double> timing_residuals_only(
    const Vec3& x,
    const std::vector<Vec3>& sensors,
    const std::vector<double>& arrival_times,
    const std::vector<double>& sensor_alts_m,
    const std::optional<double>& altitude_m) {
  auto velocities = compute_velocities(sensor_alts_m, altitude_m);
  TimingState state;
  timing_state_core(x, sensors, arrival_times, velocities, state);
  return state.residuals;
}

// Result of solving with iterative outlier rejection.
struct OutlierSolveResult {
  std::optional<ToaResult> result;
  std::vector<Vec3> used_sensors;
  std::vector<double> used_times;
  std::vector<double> used_alts;
  std::string solve_method;
};

// Iteratively solve and remove the worst-residual sensor until all are below threshold.
OutlierSolveResult solve_with_outlier_rejection(
    const std::vector<Vec3>& sensor_positions,
    const std::vector<double>& arrival_times,
    const std::vector<double>& sensor_alts_m,
    const std::optional<double>& altitude_m,
    const std::optional<Vec3>& position_prior_ecef,
    int min_sensors) {
  std::vector<Vec3> sensors = sensor_positions;
  std::vector<double> times = arrival_times;
  std::vector<double> alts = sensor_alts_m;
  std::optional<ToaResult> best_result;
  std::string best_method = "unknown";
  int n = static_cast<int>(sensors.size());
  for (int iteration = 0; iteration <= n - min_sensors; ++iteration) {
    int cur_n = static_cast<int>(sensors.size());
    std::optional<Vec3> x0;
    std::string method = "unknown";
    if (cur_n >= 5) {
      for (int shift = 0; shift < std::min(cur_n, 4); ++shift) {
        std::vector<Vec3> shifted_sensors(cur_n);
        std::vector<double> shifted_times(cur_n);
        for (int i = 0; i < cur_n; ++i) {
          shifted_sensors[i] = sensors[(i + shift) % cur_n];
          shifted_times[i] = times[(i + shift) % cur_n];
        }
        auto candidate = inamdar_5sensor(shifted_sensors, shifted_times);
        if (candidate) {
          x0 = *candidate;
          method = "inamdar_5sensor";
          break;
        }
      }
    }
    if (!x0 && cur_n >= 4 && altitude_m) {
      std::vector<Vec3> first_sensors(sensors.begin(), sensors.begin() + 4);
      std::vector<double> first_times(times.begin(), times.begin() + 4);
      auto candidate = inamdar_4sensor_altitude(first_sensors, first_times, *altitude_m);
      if (candidate) {
        x0 = *candidate;
        method = "inamdar_4sensor_alt";
      }
    }
    if (!x0 && position_prior_ecef) {
      x0 = *position_prior_ecef;
      method = "prior_aided";
    }
    if (!x0) {
      x0 = centroid_init(sensors, altitude_m);
      method = "centroid_init";
    }
    double pw = cur_n == 2 ? 8.0 : 3.0;
    std::optional<ToaResult> result;
    if (cur_n == 2 && altitude_m && position_prior_ecef) {
      result = solve_toa(sensors, times, alts, *x0, altitude_m, position_prior_ecef, 100, pw);
      if (result) {
        method = "prior_2sensor";
      }
    } else if (cur_n == 3 && altitude_m) {
      result = solve_constrained_3sensor(sensors, times, alts, *altitude_m, *x0);
      if (result && (method == "centroid_init" || method == "prior_aided" || method == "grid_search")) {
        method = "constrained_3sensor";
      }
    } else {
      result = solve_toa(sensors, times, alts, *x0, altitude_m, std::nullopt, 1000, pw);
      if (result && (method == "centroid_init" || method == "prior_aided" || method == "grid_search")) {
        method = "frisch_toa";
      }
    }
    if (!result) {
      break;
    }
    best_result = result;
    best_method = method;
    if (cur_n <= min_sensors) {
      break;
    }
    auto per_sensor = timing_residuals_only(result->position, sensors, times, alts, altitude_m);
    int worst_idx = 0;
    double worst_residual = 0.0;
    for (int i = 0; i < cur_n; ++i) {
      double abs_residual = std::abs(per_sensor[i]);
      if (abs_residual > worst_residual) {
        worst_residual = abs_residual;
        worst_idx = i;
      }
    }
    if (worst_residual < kOutlierSensorThresholdM) {
      break;
    }
    sensors.erase(sensors.begin() + worst_idx);
    times.erase(times.begin() + worst_idx);
    alts.erase(alts.begin() + worst_idx);
  }
  return {best_result, sensors, times, alts, best_method};
}

}  // namespace

// 3D Geometric Dilution of Precision from (H^T H)^{-1} trace.
double compute_gdop(const Vec3& aircraft_pos, const std::vector<Vec3>& sensor_positions) {
  if (sensor_positions.size() < 3) {
    return std::numeric_limits<double>::infinity();
  }
  std::array<std::array<double, 4>, 4> hth{};
  for (const auto& sensor : sensor_positions) {
    Vec3 diff = aircraft_pos - sensor;
    double r = norm(diff);
    if (r < 1.0) {
      return std::numeric_limits<double>::infinity();
    }
    std::array<double, 4> row = {diff.x / r, diff.y / r, diff.z / r, 1.0};
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        hth[i][j] += row[i] * row[j];
      }
    }
  }
  std::array<std::array<double, 4>, 4> inv{};
  if (!invert_matrix<4>(hth, inv)) {
    return std::numeric_limits<double>::infinity();
  }
  double trace_val = inv[0][0] + inv[1][1] + inv[2][2] + inv[3][3];
  if (trace_val < 0.0 || !std::isfinite(trace_val)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::sqrt(trace_val);
}

// 2D GDOP projected onto local East-North plane (for 2-3 sensor solves).
double compute_gdop_2d(const Vec3& aircraft_pos, const std::vector<Vec3>& sensor_positions) {
  if (sensor_positions.size() < 2) {
    return std::numeric_limits<double>::infinity();
  }
  double r_pos = norm(aircraft_pos);
  if (r_pos < 1.0) {
    return std::numeric_limits<double>::infinity();
  }
  Vec3 up = aircraft_pos / r_pos;
  Vec3 pole{0.0, 0.0, 1.0};
  Vec3 east_raw{pole.y * up.z - pole.z * up.y, pole.z * up.x - pole.x * up.z, pole.x * up.y - pole.y * up.x};
  double east_len = norm(east_raw);
  if (east_len < 1e-10) {
    return std::numeric_limits<double>::infinity();
  }
  Vec3 east = east_raw / east_len;
  Vec3 north{up.y * east.z - up.z * east.y, up.z * east.x - up.x * east.z, up.x * east.y - up.y * east.x};
  std::array<std::array<double, 2>, 2> hth{};
  for (const auto& sensor : sensor_positions) {
    Vec3 diff = aircraft_pos - sensor;
    double r = norm(diff);
    if (r < 1.0) {
      return std::numeric_limits<double>::infinity();
    }
    Vec3 unit = diff / r;
    std::array<double, 2> row = {dot(unit, east), dot(unit, north)};
    for (int i = 0; i < 2; ++i) {
      for (int j = 0; j < 2; ++j) {
        hth[i][j] += row[i] * row[j];
      }
    }
  }
  std::array<std::array<double, 2>, 2> inv{};
  if (!invert_matrix<2>(hth, inv)) {
    return std::numeric_limits<double>::infinity();
  }
  double trace_val = inv[0][0] + inv[1][1];
  if (trace_val < 0.0 || !std::isfinite(trace_val)) {
    return std::numeric_limits<double>::infinity();
  }
  return std::sqrt(trace_val);
}

// Inamdar algebraic closed-form initialiser for >=5 sensors.
// Eliminates range variable using TDOA ratios to produce a 3x3 linear system.
std::optional<Vec3> inamdar_5sensor(const std::vector<Vec3>& sensors, const std::vector<double>& arrival_times) {
  if (sensors.size() < 5 || sensors.size() != arrival_times.size()) {
    return std::nullopt;
  }
  Vec3 ref = sensors[0];
  double t_ref = arrival_times[0];
  std::vector<Vec3> r;
  std::vector<double> delta;
  for (std::size_t i = 1; i < sensors.size(); ++i) {
    r.push_back(sensors[i] - ref);
    delta.push_back((arrival_times[i] - t_ref) * kVacuumC);
  }
  if (r.size() < 4) {
    return std::nullopt;
  }
  std::array<std::array<double, 3>, 3> a{};
  std::array<double, 3> b{};
  std::array<std::pair<int, int>, 3> pairs = {{{0, 1}, {0, 2}, {0, 3}}};
  for (int row = 0; row < 3; ++row) {
    auto [k, j] = pairs[row];
    if (std::abs(delta[j]) < 1e-12) {
      return std::nullopt;
    }
    double ratio = delta[k] / delta[j];
    Vec3 row_vec = 2.0 * (r[k] - ratio * r[j]);
    a[row] = {row_vec.x, row_vec.y, row_vec.z};
    b[row] = -(delta[k] * delta[k] - ratio * delta[j] * delta[j]) + (dot(r[k], r[k]) - ratio * dot(r[j], r[j]));
  }
  std::array<double, 3> solution{};
  if (!solve_linear<3>(a, b, solution)) {
    return std::nullopt;
  }
  Vec3 position = ref + Vec3{solution[0], solution[1], solution[2]};
  for (const auto& sensor : sensors) {
    if (norm(position - sensor) > kMaxRangeM) {
      return std::nullopt;
    }
  }
  return position;
}

// Inamdar initialiser for 4 sensors with altitude constraint.
// Solves least-squares 3x3 system then snaps result to barometric altitude.
std::optional<Vec3> inamdar_4sensor_altitude(const std::vector<Vec3>& sensors, const std::vector<double>& arrival_times, double altitude_m) {
  if (sensors.size() < 4 || sensors.size() != arrival_times.size()) {
    return std::nullopt;
  }
  if (sensors.size() >= 5) {
    return inamdar_5sensor(sensors, arrival_times);
  }
  Vec3 ref = sensors[0];
  double t_ref = arrival_times[0];
  std::vector<Vec3> r;
  std::vector<double> delta;
  for (std::size_t i = 1; i < sensors.size(); ++i) {
    r.push_back(sensors[i] - ref);
    delta.push_back((arrival_times[i] - t_ref) * kVacuumC);
  }
  std::array<std::pair<int, int>, 3> pairs = {{{0, 1}, {0, 2}, {1, 2}}};
  std::vector<std::array<double, 3>> rows_a;
  std::vector<double> rows_b;
  for (const auto& [k, j] : pairs) {
    if (std::abs(delta[j]) < 1e-12) {
      continue;
    }
    double ratio = delta[k] / delta[j];
    Vec3 row_vec = 2.0 * (r[k] - ratio * r[j]);
    rows_a.push_back({row_vec.x, row_vec.y, row_vec.z});
    rows_b.push_back(-(delta[k] * delta[k] - ratio * delta[j] * delta[j]) + (dot(r[k], r[k]) - ratio * dot(r[j], r[j])));
  }
  if (rows_a.size() < 2) {
    return std::nullopt;
  }
  std::array<std::array<double, 3>, 3> ata{};
  std::array<double, 3> atb{};
  for (std::size_t row = 0; row < rows_a.size(); ++row) {
    for (int i = 0; i < 3; ++i) {
      atb[i] += rows_a[row][i] * rows_b[row];
      for (int j = 0; j < 3; ++j) {
        ata[i][j] += rows_a[row][i] * rows_a[row][j];
      }
    }
  }
  std::array<double, 3> solution{};
  if (!solve_linear<3>(ata, atb, solution)) {
    return std::nullopt;
  }
  Vec3 position = ref + Vec3{solution[0], solution[1], solution[2]};
  auto [lat, lon, alt] = ecef_to_lla(position.x, position.y, position.z);
  static_cast<void>(alt);
  position = lla_to_ecef(lat, lon, altitude_m);
  for (const auto& sensor : sensors) {
    if (norm(position - sensor) > kMaxRangeM) {
      return std::nullopt;
    }
  }
  return position;
}

// Fallback initialiser: sensor centroid, optionally snapped to altitude.
Vec3 centroid_init(const std::vector<Vec3>& sensors, std::optional<double> altitude_m) {
  Vec3 centroid{};
  for (const auto& sensor : sensors) {
    centroid += sensor;
  }
  centroid /= static_cast<double>(sensors.size());
  if (altitude_m) {
    auto [lat, lon, alt] = ecef_to_lla(centroid.x, centroid.y, centroid.z);
    static_cast<void>(alt);
    centroid = lla_to_ecef(lat, lon, *altitude_m);
  }
  return centroid;
}

constexpr int kPrior2SensorMaxNfev = 50;  // LM iteration cap for prior-aided 2-sensor

// Public wrapper: catches exceptions from the LM solver.
std::optional<ToaResult> solve_toa(
    const std::vector<Vec3>& sensors,
    const std::vector<double>& arrival_times,
    const std::vector<double>& sensor_alts_m,
    const Vec3& x0,
    std::optional<double> altitude_m,
    const std::optional<Vec3>& track_prediction_ecef,
    int max_nfev,
    double prediction_weight) {
  try {
    return solve_toa_impl(sensors, arrival_times, sensor_alts_m, x0, altitude_m, track_prediction_ecef, max_nfev, prediction_weight);
  } catch (...) {
    return std::nullopt;
  }
}

// 3-sensor solve with barometric altitude constraint.
std::optional<ToaResult> solve_constrained_3sensor(
    const std::vector<Vec3>& sensors,
    const std::vector<double>& arrival_times,
    const std::vector<double>& sensor_alts_m,
    double altitude_m,
    const Vec3& x0) {
  return solve_toa(sensors, arrival_times, sensor_alts_m, x0, altitude_m, std::nullopt, 50, 8.0);
}

// Top-level group solver: selects method based on sensor count and available priors,
// runs the solver, applies quality checks (residual, GDOP, range, prior drift).
SolveOutcome solve_group(const Group& group, const std::optional<Vec3>& position_prior_ecef, double prior_uncertainty_m) {
  SolveOutcome outcome;
  int n_sensors = static_cast<int>(group.receptions.size());
  if (n_sensors < 2) {
    outcome.fail_reason = "too_few_sensors";
    return outcome;
  }
  std::optional<double> altitude_m;
  if (group.altitude_ft) {
    altitude_m = ft_to_m(*group.altitude_ft);
  }
  if (altitude_m && position_prior_ecef) {
    if (n_sensors < kMinSensorsWithPrior) {
      outcome.fail_reason = "too_few_sensors";
      return outcome;
    }
  } else if (altitude_m) {
    if (n_sensors < kMinSensorsWithAlt) {
      outcome.fail_reason = "too_few_sensors";
      return outcome;
    }
  } else {
    if (n_sensors < kMinSensorsNoAlt) {
      outcome.fail_reason = "too_few_sensors";
      return outcome;
    }
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
  int min_sensors_needed = 0;
  if (altitude_m && position_prior_ecef) {
    min_sensors_needed = kMinSensorsWithPrior;
  } else if (altitude_m) {
    min_sensors_needed = kMinSensorsWithAlt;
  } else {
    min_sensors_needed = kMinSensorsNoAlt;
  }
  std::optional<ToaResult> result;
  std::string solve_method = "unknown";
  std::vector<Vec3> used_sensors = sensor_positions;
  std::vector<double> used_alts = sensor_alts_m;
  int n_used = n_sensors;
  if (n_sensors >= 4) {
    int outlier_min = std::max(3, min_sensors_needed);
    auto outlier_result = solve_with_outlier_rejection(sensor_positions, arrival_times, sensor_alts_m, altitude_m, position_prior_ecef, outlier_min);
    result = outlier_result.result;
    solve_method = outlier_result.solve_method;
    used_sensors = outlier_result.used_sensors;
    used_alts = outlier_result.used_alts;
    n_used = static_cast<int>(used_sensors.size());
    if (n_used < n_sensors) {
      solve_method += "_drop" + std::to_string(n_sensors - n_used);
    }
  } else {
    std::optional<Vec3> x0;
    if (position_prior_ecef) {
      x0 = *position_prior_ecef;
      solve_method = "prior_aided";
    }
    if (!x0) {
      x0 = centroid_init(sensor_positions, altitude_m);
      solve_method = "centroid_init";
     }
     double pw = n_sensors == 2 ? clamp(500.0 / std::max(prior_uncertainty_m, 1.0), 1.0, 12.0) : 3.0;
     if (n_sensors == 2 && altitude_m && position_prior_ecef) {
       result = solve_toa(sensor_positions, arrival_times, sensor_alts_m, *x0, altitude_m, position_prior_ecef, kPrior2SensorMaxNfev, pw);
       if (result) {
         solve_method = "prior_2sensor";
       }
     } else if (n_sensors == 3 && altitude_m) {
      result = solve_constrained_3sensor(sensor_positions, arrival_times, sensor_alts_m, *altitude_m, *x0);
      if (result && (solve_method == "centroid_init" || solve_method == "prior_aided" || solve_method == "grid_search")) {
        solve_method = "constrained_3sensor";
      }
    } else {
      result = solve_toa(sensor_positions, arrival_times, sensor_alts_m, *x0, altitude_m, std::nullopt, 1000, pw);
      if (result && (solve_method == "centroid_init" || solve_method == "prior_aided" || solve_method == "grid_search")) {
        solve_method = "frisch_toa";
      }
    }
  }
  if (!result) {
    outcome.fail_reason = "solver_diverged";
    return outcome;
  }
  Vec3 position = result->position;
  double residual_m = result->residual_m;
  double prior_offset_m = 0.0;
  if (position_prior_ecef) {
    prior_offset_m = norm(position - *position_prior_ecef);
  }
  double quality_residual_m = (n_sensors == 2 && position_prior_ecef) ? std::max(residual_m, prior_offset_m) : residual_m;
  for (const auto& sensor : used_sensors) {
    if (norm(position - sensor) > kMaxRangeM) {
      outcome.fail_reason = "range_exceeded";
      return outcome;
    }
  }
  double effective_max_residual = kMaxResidualM;
  if (n_sensors == 2 && position_prior_ecef) {
    effective_max_residual = kMaxResidualPriorM;
  } else if (n_used == 3 && altitude_m) {
    effective_max_residual = 2000.0;
  } else if (n_used >= 4) {
    effective_max_residual = 5000.0;
  }
  if (residual_m > effective_max_residual) {
    outcome.fail_reason = "residual_exceeded";
    return outcome;
  }
  if (position_prior_ecef && n_sensors <= 3) {
    double base_drift = n_sensors == 2 ? kMaxPriorDrift2SensorM : kMaxPriorDriftM;
    double drift_scale = std::min(3.0, std::max(1.0, kMaxResidualPriorM / std::max(residual_m, 10.0)));
    double max_prior_drift = base_drift * drift_scale;
    if (prior_offset_m > max_prior_drift) {
      outcome.fail_reason = "prior_drift_exceeded";
      return outcome;
    }
  }
  double gdop = 0.0;
  if (n_used >= 4) {
    gdop = compute_gdop(position, used_sensors);
    if (gdop > kMaxGdop) {
      outcome.fail_reason = "gdop_exceeded";
      return outcome;
    }
  } else if (n_used >= 2 && altitude_m) {
    gdop = compute_gdop_2d(position, used_sensors);
    if (n_used >= 3 && gdop > kMaxGdop2d) {
      outcome.fail_reason = "gdop_2d_exceeded";
      return outcome;
    }
  }
  auto [lat, lon, solved_alt_m] = ecef_to_lla(position.x, position.y, position.z);
  double final_alt_ft = group.altitude_ft ? *group.altitude_ft : m_to_ft(solved_alt_m);
  SolveFix fix;
  fix.icao = group.icao;
  fix.lat = lat;
  fix.lon = lon;
  fix.alt_ft = final_alt_ft;
  fix.residual_m = residual_m;
  fix.quality_residual_m = quality_residual_m;
  fix.gdop = gdop;
  fix.num_sensors = n_used;
  fix.solve_method = solve_method;
  fix.timestamp_s = ref_timestamp_s;
  fix.timestamp_ns = ref_timestamp_ns;
  fix.df_type = group.df_type;
  fix.squawk = group.squawk;
  fix.raw_msg = group.raw_msg;
  fix.t0_s = result->t0_s;
  outcome.result = fix;
  return outcome;
}

}
