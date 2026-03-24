#include "core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>

namespace native_l45 {

namespace {

constexpr double kMaxPairVarianceUs2 = 50.0;

double monotonic_now() {
  using clock = std::chrono::steady_clock;
  return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

int delegate_2sensor_min_solves() {
  static const int value = []() {
    const char* env = std::getenv("NATIVE_L45_DELEGATE_2SENSOR_MIN_SOLVES");
    if (!env || !*env) {
      return 0;
    }
    char* end = nullptr;
    long parsed = std::strtol(env, &end, 10);
    if (end == env || (end && *end != '\0')) {
      return 0;
    }
    if (parsed < 0) {
      return 0;
    }
    if (parsed > static_cast<long>(std::numeric_limits<int>::max())) {
      return std::numeric_limits<int>::max();
    }
    return static_cast<int>(parsed);
  }();
  return value;
}

std::optional<int> extract_df17_altitude(const std::string& raw_msg) {
  auto frame = extract_df17_position_fields(raw_msg);
  if (!frame || !frame->alt_ft) {
    return std::nullopt;
  }
  return frame->alt_ft;
}

std::string cpr_stats_json(const CPRBuffer& buffer) {
  std::ostringstream os;
  os << '{';
  os << "\"frames_received\":" << buffer.frames_received;
  os << ",\"global_decodes\":" << buffer.global_decodes;
  os << ",\"local_decodes\":" << buffer.local_decodes;
  os << ",\"tracked_icaos\":" << buffer.tracked_icaos();
  os << '}';
  return os.str();
}

}  // namespace

Layer4Processor::Layer4Processor() : override_map_(load_location_overrides()) {}

bool Layer4Processor::process_line(const std::string& line, std::ostream& out) {
  Group group;
  if (!parse_group_json(line, group)) {
    ++stats_.parse_errors;
    return false;
  }
  ++stats_.groups_received;
  process_group(std::move(group), out);
  return true;
}

void Layer4Processor::process_group(Group group, std::ostream& out) {
  double now = monotonic_now();
  auto receptions = group.receptions;
  if (override_map_ && !override_map_->empty()) {
    receptions = apply_overrides(receptions, *override_map_);
    if (!receptions.empty()) {
      group.receptions = receptions;
      group.num_sensors = static_cast<int>(receptions.size());
    }
  }
  int n_sensors = static_cast<int>(receptions.size());
  if (n_sensors < 2) {
    ++stats_.groups_skipped_sensors;
    return;
  }
  std::string icao = group.icao;
  std::string raw_msg = group.raw_msg;
  std::optional<double> msg_timestamp;
  if (!receptions.empty()) {
    msg_timestamp = static_cast<double>(receptions[0].timestamp_s) + static_cast<double>(receptions[0].timestamp_ns) * 1e-9;
  }
  if (group.df_type == 17 && raw_msg.size() == 28) {
    auto cpr_frame = extract_df17_position_fields(raw_msg);
    if (cpr_frame && msg_timestamp) {
      cpr_frame->timestamp = *msg_timestamp;
      auto decoded = cpr_buffer_.add_frame(icao, *cpr_frame);
      if (decoded) {
        std::optional<double> alt_m;
        if (decoded->alt_ft) {
          alt_m = ft_to_m(static_cast<double>(*decoded->alt_ft));
        } else {
          auto cached = pos_cache_.get(icao, *msg_timestamp);
          if (cached) {
            alt_m = cached->alt_m;
          }
        }
        Vec3 decoded_ecef;
        if (alt_m) {
          decoded_ecef = position_to_ecef(decoded->lat, decoded->lon, decoded->alt_ft);
        } else {
          decoded_ecef = position_to_ecef(decoded->lat, decoded->lon, std::nullopt);
          alt_m = 10000.0;
        }
        pos_cache_.put(icao, decoded_ecef, decoded->lat, decoded->lon, *alt_m, *msg_timestamp, 0.0);
        ++cpr_seeds_;
        clock_cal_.process_adsb_reference(decoded_ecef, receptions, now);
      }
    }
  }
  if (group.df_type == 17 && raw_msg.size() == 28 && msg_timestamp) {
    auto velocity = extract_df17_velocity(raw_msg);
    if (velocity) {
      constexpr double kKtsToMps = 0.514444;
      pos_cache_.update_velocity_from_adsb(
          icao,
          velocity->ew_knots * kKtsToMps,
          velocity->ns_knots * kKtsToMps,
          static_cast<double>(velocity->vrate_fpm) * 0.00508,
          *msg_timestamp);
    }
  }
  std::optional<double> effective_alt_ft = group.altitude_ft;
  if (!effective_alt_ft && group.df_type == 17) {
    auto df17_alt = extract_df17_altitude(raw_msg);
    if (df17_alt) {
      effective_alt_ft = static_cast<double>(*df17_alt);
      ++df17_alt_extracted_;
    }
  }
  if (!effective_alt_ft && msg_timestamp) {
    auto cached = pos_cache_.get(icao, *msg_timestamp);
    if (cached) {
      effective_alt_ft = std::round(cached->alt_m / 0.3048);
      ++cached_alt_used_;
    }
  }
  Group working_group = group;
  if (effective_alt_ft != group.altitude_ft) {
    working_group.altitude_ft = effective_alt_ft;
  }
  std::vector<Reception> corrected_receptions = receptions;
  Group corrected_group = working_group;
  if (clock_cal_.has_any_calibration(receptions)) {
    corrected_receptions = clock_cal_.correct_timestamps(receptions, now);
    corrected_group.receptions = corrected_receptions;
    corrected_group.num_sensors = static_cast<int>(corrected_receptions.size());
  }
  std::optional<Vec3> position_prior;
  auto cached = (msg_timestamp ? pos_cache_.get(icao, *msg_timestamp) : std::optional<CachedPosition>{});
  if (cached && msg_timestamp) {
    position_prior = cached->predict(*msg_timestamp);
  }
  if (n_sensors == 2 && effective_alt_ft) {
    if (!position_prior) {
      out << to_json_unsolved_group(group) << '\n';
      ++stats_.groups_skipped_sensors;
      ++stats_.failure_reasons["no_prior_2sensor"];
      return;
    }
    int delegate_min_solves = delegate_2sensor_min_solves();
    if (delegate_min_solves > 0 && cached && cached->solve_count >= delegate_min_solves && cached->velocity_ecef && cached->residual_m > 0.0) {
      out << to_json_unsolved_group(corrected_group) << '\n';
      ++stats_.groups_failed;
      ++stats_.failure_reasons["delegated_2sensor_prediction"];
      return;
    }
    if (corrected_receptions.size() == 2) {
      auto key = ClockCalibrator::pair_key(corrected_receptions[0].sensor_id, corrected_receptions[1].sensor_id);
      auto it = clock_cal_.pairings.find(key);
      if (it != clock_cal_.pairings.end() && it->second.valid) {
        double pair_var_us2 = it->second.variance * 1e12;
        if (pair_var_us2 > kMaxPairVarianceUs2) {
          out << to_json_unsolved_group(group) << '\n';
          ++stats_.groups_skipped_sensors;
          ++stats_.failure_reasons["pair_variance_exceeded"];
          return;
        }
      }
    }
  }
  auto outcome = solve_group(corrected_group, position_prior);
  if (outcome.result) {
    Vec3 aircraft_ecef = lla_to_ecef(outcome.result->lat, outcome.result->lon, ft_to_m(outcome.result->alt_ft));
    if (cached && msg_timestamp) {
      if (!cached->is_physically_consistent(aircraft_ecef, *msg_timestamp, outcome.result->quality_residual_m)) {
        ++stats_.groups_failed;
        ++stats_.failure_reasons["physically_inconsistent"];
        return;
      }
    }
    out << to_json_fix(*outcome.result) << '\n';
    stats_.record_solve(outcome.result->solve_method, outcome.result->residual_m);
    if (msg_timestamp) {
      pos_cache_.put(icao, aircraft_ecef, outcome.result->lat, outcome.result->lon, ft_to_m(outcome.result->alt_ft), *msg_timestamp, outcome.result->quality_residual_m);
    }
    if (outcome.result->quality_residual_m < 200.0 && clock_cal_.has_any_calibration(receptions)) {
      clock_cal_.process_adsb_reference(aircraft_ecef, receptions, now);
    }
    for (const auto& rec : corrected_receptions) {
      double dt_s = static_cast<double>(rec.timestamp_s - outcome.result->timestamp_s);
      double dt_ns = static_cast<double>(rec.timestamp_ns - outcome.result->timestamp_ns);
      double arr_time = dt_s + dt_ns * 1e-9;
      double dist = norm(aircraft_ecef - rec.sensor_ecef);
      double expected_arr = dist / kVacuumC;
      double actual_arr = arr_time - outcome.result->t0_s;
      double residual_m = (actual_arr - expected_arr) * kVacuumC;
      stats_.record_sensor_residual(rec.sensor_id, residual_m);
    }
  } else {
    ++stats_.groups_failed;
    if (!outcome.fail_reason.empty()) {
      ++stats_.failure_reasons[outcome.fail_reason];
    }
    if (n_sensors == 2 && effective_alt_ft) {
      out << to_json_unsolved_group(group) << '\n';
    }
  }
}

void Layer4Processor::finish(std::ostream& log) const {
  log << "=== MLAT Solver (Layer 4) — Native ===\n";
  log << "Shutting down. Final stats:\n";
  std::ostringstream stats_json;
  stats_json << stats_.to_json();
  std::string base = stats_json.str();
  if (!base.empty() && base.back() == '}') {
    base.pop_back();
  }
  log << "[stats] " << base;
  log << ",\"clock_cal\":" << clock_cal_.stats_json();
  log << ",\"pos_cache\":" << pos_cache_.stats_json();
  log << ",\"cpr_buffer\":" << cpr_stats_json(cpr_buffer_);
  log << ",\"cpr_cache_seeds\":" << cpr_seeds_;
  log << ",\"df17_alt_extracted\":" << df17_alt_extracted_;
  log << ",\"cached_alt_used\":" << cached_alt_used_;
  if (override_map_) {
    log << ",\"location_overrides\":" << override_map_->stats_json();
  }
  log << "}\n";
  log << "Clock calibration: " << clock_cal_.calibrated_pairs << " valid pairings\n";
  log << "Position cache: " << pos_cache_.size() << " aircraft tracked\n";
  log << "CPR cache seeds: " << cpr_seeds_ << " (global=" << cpr_buffer_.global_decodes << ", local=" << cpr_buffer_.local_decodes << ")\n";
}

int run_layer4(std::istream& in, std::ostream& out, std::ostream& log) {
  Layer4Processor processor;
  std::string line;
  while (std::getline(in, line)) {
    if (line.empty()) {
      continue;
    }
    processor.process_line(line, out);
  }
  processor.finish(log);
  return 0;
}

}
