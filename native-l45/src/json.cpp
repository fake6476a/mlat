#include "core.hpp"

#include <charconv>
#include <cctype>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace native_l45 {

namespace {

struct FastCursor {
  const char* cur = nullptr;
  const char* end = nullptr;
};

void fast_skip_ws(FastCursor& cursor) {
  while (cursor.cur < cursor.end && std::isspace(static_cast<unsigned char>(*cursor.cur))) {
    ++cursor.cur;
  }
}

bool fast_consume(FastCursor& cursor, char expected) {
  fast_skip_ws(cursor);
  if (cursor.cur >= cursor.end || *cursor.cur != expected) {
    return false;
  }
  ++cursor.cur;
  return true;
}

bool fast_parse_literal(FastCursor& cursor, std::string_view literal) {
  fast_skip_ws(cursor);
  if (static_cast<std::size_t>(cursor.end - cursor.cur) < literal.size()) {
    return false;
  }
  if (std::string_view(cursor.cur, literal.size()) != literal) {
    return false;
  }
  cursor.cur += literal.size();
  return true;
}

bool fast_parse_string_view(FastCursor& cursor, std::string_view& out) {
  fast_skip_ws(cursor);
  if (cursor.cur >= cursor.end || *cursor.cur != '"') {
    return false;
  }
  ++cursor.cur;
  const char* start = cursor.cur;
  while (cursor.cur < cursor.end) {
    char c = *cursor.cur;
    if (c == '"') {
      out = std::string_view(start, static_cast<std::size_t>(cursor.cur - start));
      ++cursor.cur;
      return true;
    }
    if (c == '\\' || static_cast<unsigned char>(c) < 0x20) {
      return false;
    }
    ++cursor.cur;
  }
  return false;
}

bool fast_parse_string(FastCursor& cursor, std::string& out) {
  std::string_view view;
  if (!fast_parse_string_view(cursor, view)) {
    return false;
  }
  out.assign(view.data(), view.size());
  return true;
}

template <typename T>
bool fast_parse_number(FastCursor& cursor, T& out) {
  fast_skip_ws(cursor);
  auto result = std::from_chars(cursor.cur, cursor.end, out);
  if (result.ec != std::errc()) {
    return false;
  }
  cursor.cur = result.ptr;
  return true;
}

bool fast_parse_optional_string(FastCursor& cursor, std::optional<std::string>& out) {
  fast_skip_ws(cursor);
  if (cursor.cur < cursor.end && *cursor.cur == 'n') {
    if (!fast_parse_literal(cursor, "null")) {
      return false;
    }
    out = std::nullopt;
    return true;
  }
  std::string value;
  if (!fast_parse_string(cursor, value)) {
    return false;
  }
  out = std::move(value);
  return true;
}

bool fast_parse_optional_double(FastCursor& cursor, std::optional<double>& out) {
  fast_skip_ws(cursor);
  if (cursor.cur < cursor.end && *cursor.cur == 'n') {
    if (!fast_parse_literal(cursor, "null")) {
      return false;
    }
    out = std::nullopt;
    return true;
  }
  double value = 0.0;
  if (!fast_parse_number(cursor, value)) {
    return false;
  }
  out = value;
  return true;
}

bool fast_parse_reception(FastCursor& cursor, Reception& out) {
  if (!fast_consume(cursor, '{')) {
    return false;
  }
  bool have_sensor_id = false;
  bool have_lat = false;
  bool have_lon = false;
  bool have_alt = false;
  bool have_timestamp_s = false;
  bool have_timestamp_ns = false;
  out = Reception{};
  while (true) {
    std::string_view key;
    if (!fast_parse_string_view(cursor, key) || !fast_consume(cursor, ':')) {
      return false;
    }
    if (key == "sensor_id") {
      have_sensor_id = fast_parse_number(cursor, out.sensor_id);
    } else if (key == "lat") {
      have_lat = fast_parse_number(cursor, out.lat);
    } else if (key == "lon") {
      have_lon = fast_parse_number(cursor, out.lon);
    } else if (key == "alt") {
      have_alt = fast_parse_number(cursor, out.alt);
    } else if (key == "timestamp_s") {
      have_timestamp_s = fast_parse_number(cursor, out.timestamp_s);
    } else if (key == "timestamp_ns") {
      have_timestamp_ns = fast_parse_number(cursor, out.timestamp_ns);
    } else {
      return false;
    }
    if (!(have_sensor_id || have_lat || have_lon || have_alt || have_timestamp_s || have_timestamp_ns)) {
      return false;
    }
    fast_skip_ws(cursor);
    if (cursor.cur >= cursor.end) {
      return false;
    }
    if (*cursor.cur == '}') {
      ++cursor.cur;
      break;
    }
    if (*cursor.cur != ',') {
      return false;
    }
    ++cursor.cur;
  }
  if (!have_sensor_id || !have_lat || !have_lon || !have_alt || !have_timestamp_s || !have_timestamp_ns) {
    return false;
  }
  out.sensor_ecef = sensor_lla_to_ecef(out.lat, out.lon, out.alt);
  return true;
}

bool fast_parse_receptions(FastCursor& cursor, std::vector<Reception>& out) {
  if (!fast_consume(cursor, '[')) {
    return false;
  }
  out.clear();
  fast_skip_ws(cursor);
  if (cursor.cur < cursor.end && *cursor.cur == ']') {
    ++cursor.cur;
    return true;
  }
  while (true) {
    Reception rec;
    if (!fast_parse_reception(cursor, rec)) {
      return false;
    }
    out.push_back(rec);
    fast_skip_ws(cursor);
    if (cursor.cur >= cursor.end) {
      return false;
    }
    if (*cursor.cur == ']') {
      ++cursor.cur;
      return true;
    }
    if (*cursor.cur != ',') {
      return false;
    }
    ++cursor.cur;
  }
}

bool parse_group_json_fast(std::string_view line, Group& out) {
  FastCursor cursor{line.data(), line.data() + line.size()};
  if (!fast_consume(cursor, '{')) {
    return false;
  }
  out = Group{};
  bool have_icao = false;
  bool have_df_type = false;
  bool have_raw_msg = false;
  bool have_receptions = false;
  while (true) {
    std::string_view key;
    if (!fast_parse_string_view(cursor, key) || !fast_consume(cursor, ':')) {
      return false;
    }
    bool parsed = false;
    if (key == "icao") {
      parsed = fast_parse_string(cursor, out.icao);
      have_icao = parsed;
    } else if (key == "df_type") {
      parsed = fast_parse_number(cursor, out.df_type);
      have_df_type = parsed;
    } else if (key == "altitude_ft") {
      parsed = fast_parse_optional_double(cursor, out.altitude_ft);
    } else if (key == "squawk") {
      parsed = fast_parse_optional_string(cursor, out.squawk);
    } else if (key == "raw_msg") {
      parsed = fast_parse_string(cursor, out.raw_msg);
      have_raw_msg = parsed;
    } else if (key == "num_sensors") {
      parsed = fast_parse_number(cursor, out.num_sensors);
    } else if (key == "receptions") {
      parsed = fast_parse_receptions(cursor, out.receptions);
      have_receptions = parsed;
    } else {
      return false;
    }
    if (!parsed) {
      return false;
    }
    fast_skip_ws(cursor);
    if (cursor.cur >= cursor.end) {
      return false;
    }
    if (*cursor.cur == '}') {
      ++cursor.cur;
      break;
    }
    if (*cursor.cur != ',') {
      return false;
    }
    ++cursor.cur;
  }
  fast_skip_ws(cursor);
  if (cursor.cur != cursor.end || !have_icao || !have_df_type || !have_raw_msg || !have_receptions) {
    return false;
  }
  if (out.num_sensors == 0) {
    out.num_sensors = static_cast<int>(out.receptions.size());
  }
  return true;
}

std::optional<Reception> parse_reception_object(const JsonValue::Object& object) {
  auto sensor_id = json_get_int_optional(object, "sensor_id");
  auto lat = json_get_number_optional(object, "lat");
  auto lon = json_get_number_optional(object, "lon");
  auto alt = json_get_number_optional(object, "alt");
  auto timestamp_s = json_get_int_optional(object, "timestamp_s");
  auto timestamp_ns = json_get_int_optional(object, "timestamp_ns");
  if (!sensor_id || !lat || !lon || !alt || !timestamp_s || !timestamp_ns) {
    return std::nullopt;
  }
  Reception reception;
  reception.sensor_id = *sensor_id;
  reception.lat = *lat;
  reception.lon = *lon;
  reception.alt = *alt;
  reception.sensor_ecef = sensor_lla_to_ecef(*lat, *lon, *alt);
  reception.timestamp_s = *timestamp_s;
  reception.timestamp_ns = *timestamp_ns;
  return reception;
}

bool parse_group_object(const JsonValue::Object& object, Group& out) {
  auto icao = json_get_string_optional(object, "icao");
  auto df_type = json_get_int_optional(object, "df_type");
  auto raw_msg = json_get_string_optional(object, "raw_msg");
  if (!icao || !df_type || !raw_msg) {
    return false;
  }
  out = Group{};
  out.icao = *icao;
  out.df_type = static_cast<int>(*df_type);
  out.raw_msg = *raw_msg;
  out.altitude_ft = json_get_number_optional(object, "altitude_ft");
  out.squawk = json_get_string_optional(object, "squawk");
  auto num_sensors = json_get_int_optional(object, "num_sensors");
  out.num_sensors = num_sensors ? static_cast<int>(*num_sensors) : 0;
  const JsonValue* receptions = json_find(object, "receptions");
  if (receptions == nullptr || !receptions->is_array()) {
    return false;
  }
  for (const auto& item : receptions->as_array()) {
    if (!item.is_object()) {
      return false;
    }
    auto parsed = parse_reception_object(item.as_object());
    if (!parsed) {
      return false;
    }
    out.receptions.push_back(*parsed);
  }
  if (out.num_sensors == 0) {
    out.num_sensors = static_cast<int>(out.receptions.size());
  }
  return true;
}

bool parse_fix_object(const JsonValue::Object& object, SolveFix& out) {
  auto icao = json_get_string_optional(object, "icao");
  auto lat = json_get_number_optional(object, "lat");
  auto lon = json_get_number_optional(object, "lon");
  auto alt_ft = json_get_number_optional(object, "alt_ft");
  auto residual_m = json_get_number_optional(object, "residual_m");
  auto quality_residual_m = json_get_number_optional(object, "quality_residual_m");
  auto gdop = json_get_number_optional(object, "gdop");
  auto num_sensors = json_get_int_optional(object, "num_sensors");
  auto solve_method = json_get_string_optional(object, "solve_method");
  auto timestamp_s = json_get_int_optional(object, "timestamp_s");
  auto timestamp_ns = json_get_int_optional(object, "timestamp_ns");
  auto df_type = json_get_int_optional(object, "df_type");
  auto raw_msg = json_get_string_optional(object, "raw_msg");
  auto t0_s = json_get_number_optional(object, "t0_s");
  if (!icao || !lat || !lon || !alt_ft || !residual_m || !quality_residual_m || !gdop || !num_sensors || !solve_method || !timestamp_s || !timestamp_ns || !df_type || !raw_msg || !t0_s) {
    return false;
  }
  out = SolveFix{};
  out.icao = *icao;
  out.lat = *lat;
  out.lon = *lon;
  out.alt_ft = *alt_ft;
  out.residual_m = *residual_m;
  out.quality_residual_m = *quality_residual_m;
  out.gdop = *gdop;
  out.num_sensors = static_cast<int>(*num_sensors);
  out.solve_method = *solve_method;
  out.timestamp_s = *timestamp_s;
  out.timestamp_ns = *timestamp_ns;
  out.df_type = static_cast<int>(*df_type);
  out.squawk = json_get_string_optional(object, "squawk");
  out.raw_msg = *raw_msg;
  out.t0_s = *t0_s;
  return true;
}

void append_json_string(std::ostringstream& os, std::string_view value) {
  os << '"' << json_escape(value) << '"';
}

std::string format_double(double value) {
  if (!std::isfinite(value)) {
    return "null";
  }
  std::ostringstream os;
  os << std::setprecision(15) << value;
  return os.str();
}

std::string group_json_object(const Group& group) {
  std::ostringstream os;
  os << '{';
  os << "\"icao\":";
  append_json_string(os, group.icao);
  os << ",\"df_type\":" << group.df_type;
  os << ",\"altitude_ft\":";
  if (group.altitude_ft) {
    os << format_double(*group.altitude_ft);
  } else {
    os << "null";
  }
  os << ",\"squawk\":";
  if (group.squawk) {
    append_json_string(os, *group.squawk);
  } else {
    os << "null";
  }
  os << ",\"raw_msg\":";
  append_json_string(os, group.raw_msg);
  os << ",\"num_sensors\":" << group.num_sensors;
  os << ",\"receptions\":[";
  for (std::size_t i = 0; i < group.receptions.size(); ++i) {
    const auto& rec = group.receptions[i];
    if (i > 0) {
      os << ',';
    }
    os << '{';
    os << "\"sensor_id\":" << rec.sensor_id;
    os << ",\"lat\":" << format_double(rec.lat);
    os << ",\"lon\":" << format_double(rec.lon);
    os << ",\"alt\":" << format_double(rec.alt);
    os << ",\"timestamp_s\":" << rec.timestamp_s;
    os << ",\"timestamp_ns\":" << rec.timestamp_ns;
    os << '}';
  }
  os << "]}";
  return os.str();
}

std::string squawk_json(const std::optional<std::string>& squawk) {
  if (!squawk) {
    return "null";
  }
  return "\"" + json_escape(*squawk) + "\"";
}

}  // namespace

bool JsonValue::is_null() const { return std::holds_alternative<std::nullptr_t>(storage); }
bool JsonValue::is_bool() const { return std::holds_alternative<bool>(storage); }
bool JsonValue::is_number() const { return std::holds_alternative<double>(storage); }
bool JsonValue::is_string() const { return std::holds_alternative<std::string>(storage); }
bool JsonValue::is_array() const { return std::holds_alternative<Array>(storage); }
bool JsonValue::is_object() const { return std::holds_alternative<Object>(storage); }
bool JsonValue::as_bool(bool fallback) const { return is_bool() ? std::get<bool>(storage) : fallback; }
double JsonValue::as_number(double fallback) const { return is_number() ? std::get<double>(storage) : fallback; }
const std::string& JsonValue::as_string() const { return std::get<std::string>(storage); }
const JsonValue::Array& JsonValue::as_array() const { return std::get<Array>(storage); }
const JsonValue::Object& JsonValue::as_object() const { return std::get<Object>(storage); }

JsonParser::JsonParser(std::string_view input) : input_(input) {}

JsonValue JsonParser::parse() {
  skip_ws();
  JsonValue value = parse_value();
  skip_ws();
  if (pos_ != input_.size()) {
    throw std::runtime_error("trailing characters");
  }
  return value;
}

void JsonParser::skip_ws() {
  while (pos_ < input_.size() && std::isspace(static_cast<unsigned char>(input_[pos_]))) {
    ++pos_;
  }
}

char JsonParser::peek() const {
  return pos_ < input_.size() ? input_[pos_] : '\0';
}

char JsonParser::get() {
  if (pos_ >= input_.size()) {
    throw std::runtime_error("unexpected end of input");
  }
  return input_[pos_++];
}

bool JsonParser::consume(char c) {
  if (peek() == c) {
    ++pos_;
    return true;
  }
  return false;
}

void JsonParser::expect(char c) {
  char got = get();
  if (got != c) {
    throw std::runtime_error("unexpected token");
  }
}

JsonValue JsonParser::parse_value() {
  skip_ws();
  char c = peek();
  if (c == '{') {
    return parse_object();
  }
  if (c == '[') {
    return parse_array();
  }
  if (c == '"') {
    return parse_string();
  }
  if (c == 'n') {
    return parse_null();
  }
  if (c == 't') {
    return parse_true();
  }
  if (c == 'f') {
    return parse_false();
  }
  return parse_number();
}

JsonValue JsonParser::parse_null() {
  if (input_.substr(pos_, 4) != "null") {
    throw std::runtime_error("invalid null");
  }
  pos_ += 4;
  return JsonValue{JsonValue::Storage{nullptr}};
}

JsonValue JsonParser::parse_true() {
  if (input_.substr(pos_, 4) != "true") {
    throw std::runtime_error("invalid true");
  }
  pos_ += 4;
  return JsonValue{JsonValue::Storage{true}};
}

JsonValue JsonParser::parse_false() {
  if (input_.substr(pos_, 5) != "false") {
    throw std::runtime_error("invalid false");
  }
  pos_ += 5;
  return JsonValue{JsonValue::Storage{false}};
}

JsonValue JsonParser::parse_number() {
  std::size_t start = pos_;
  if (peek() == '-') {
    ++pos_;
  }
  while (std::isdigit(static_cast<unsigned char>(peek()))) {
    ++pos_;
  }
  if (peek() == '.') {
    ++pos_;
    while (std::isdigit(static_cast<unsigned char>(peek()))) {
      ++pos_;
    }
  }
  if (peek() == 'e' || peek() == 'E') {
    ++pos_;
    if (peek() == '+' || peek() == '-') {
      ++pos_;
    }
    while (std::isdigit(static_cast<unsigned char>(peek()))) {
      ++pos_;
    }
  }
  std::string token(input_.substr(start, pos_ - start));
  return JsonValue{JsonValue::Storage{std::stod(token)}};
}

int JsonParser::hex_value(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return 10 + c - 'a';
  if (c >= 'A' && c <= 'F') return 10 + c - 'A';
  return -1;
}

JsonValue JsonParser::parse_string() {
  expect('"');
  std::string out;
  while (true) {
    char c = get();
    if (c == '"') {
      break;
    }
    if (c != '\\') {
      out.push_back(c);
      continue;
    }
    char esc = get();
    switch (esc) {
      case '"': out.push_back('"'); break;
      case '\\': out.push_back('\\'); break;
      case '/': out.push_back('/'); break;
      case 'b': out.push_back('\b'); break;
      case 'f': out.push_back('\f'); break;
      case 'n': out.push_back('\n'); break;
      case 'r': out.push_back('\r'); break;
      case 't': out.push_back('\t'); break;
      case 'u': {
        int code = 0;
        for (int i = 0; i < 4; ++i) {
          int hv = hex_value(get());
          if (hv < 0) {
            throw std::runtime_error("invalid unicode escape");
          }
          code = (code << 4) | hv;
        }
        if (code < 0x80) {
          out.push_back(static_cast<char>(code));
        } else if (code < 0x800) {
          out.push_back(static_cast<char>(0xC0 | (code >> 6)));
          out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
        } else {
          out.push_back(static_cast<char>(0xE0 | (code >> 12)));
          out.push_back(static_cast<char>(0x80 | ((code >> 6) & 0x3F)));
          out.push_back(static_cast<char>(0x80 | (code & 0x3F)));
        }
        break;
      }
      default:
        throw std::runtime_error("invalid escape");
    }
  }
  return JsonValue{JsonValue::Storage{std::move(out)}};
}

JsonValue JsonParser::parse_array() {
  expect('[');
  JsonValue::Array out;
  skip_ws();
  if (consume(']')) {
    return JsonValue{JsonValue::Storage{std::move(out)}};
  }
  while (true) {
    out.push_back(parse_value());
    skip_ws();
    if (consume(']')) {
      break;
    }
    expect(',');
    skip_ws();
  }
  return JsonValue{JsonValue::Storage{std::move(out)}};
}

JsonValue JsonParser::parse_object() {
  expect('{');
  JsonValue::Object out;
  skip_ws();
  if (consume('}')) {
    return JsonValue{JsonValue::Storage{std::move(out)}};
  }
  while (true) {
    skip_ws();
    JsonValue key = parse_string();
    skip_ws();
    expect(':');
    skip_ws();
    out.emplace_back(key.as_string(), parse_value());
    skip_ws();
    if (consume('}')) {
      break;
    }
    expect(',');
    skip_ws();
  }
  return JsonValue{JsonValue::Storage{std::move(out)}};
}

const JsonValue* json_find(const JsonValue::Object& object, std::string_view key) {
  for (const auto& [name, value] : object) {
    if (name == key) {
      return &value;
    }
  }
  return nullptr;
}

std::optional<std::string> json_get_string_optional(const JsonValue::Object& object, std::string_view key) {
  const JsonValue* value = json_find(object, key);
  if (value == nullptr || value->is_null() || !value->is_string()) {
    return std::nullopt;
  }
  return value->as_string();
}

std::optional<double> json_get_number_optional(const JsonValue::Object& object, std::string_view key) {
  const JsonValue* value = json_find(object, key);
  if (value == nullptr || value->is_null() || !value->is_number()) {
    return std::nullopt;
  }
  return value->as_number();
}

std::optional<std::int64_t> json_get_int_optional(const JsonValue::Object& object, std::string_view key) {
  auto value = json_get_number_optional(object, key);
  if (!value) {
    return std::nullopt;
  }
  return static_cast<std::int64_t>(std::llround(*value));
}

std::optional<bool> json_get_bool_optional(const JsonValue::Object& object, std::string_view key) {
  const JsonValue* value = json_find(object, key);
  if (value == nullptr || value->is_null() || !value->is_bool()) {
    return std::nullopt;
  }
  return value->as_bool();
}

bool parse_group_json(std::string_view line, Group& out) {
  if (parse_group_json_fast(line, out)) {
    return true;
  }
  try {
    JsonParser parser(line);
    JsonValue root = parser.parse();
    if (!root.is_object()) {
      return false;
    }
    return parse_group_object(root.as_object(), out);
  } catch (...) {
    return false;
  }
}

bool parse_layer4_record_json(std::string_view line, Layer4Record& out) {
  try {
    JsonParser parser(line);
    JsonValue root = parser.parse();
    if (!root.is_object()) {
      return false;
    }
    const auto& object = root.as_object();
    const JsonValue* unsolved = json_find(object, "unsolved_group");
    if (unsolved != nullptr) {
      if (!unsolved->is_object()) {
        return false;
      }
      out = Layer4Record{};
      out.is_unsolved_group = true;
      return parse_group_object(unsolved->as_object(), out.unsolved_group);
    }
    out = Layer4Record{};
    out.is_unsolved_group = false;
    return parse_fix_object(object, out.fix);
  } catch (...) {
    return false;
  }
}

std::string to_json_unsolved_group(const Group& group) {
  return std::string{"{\"unsolved_group\":"} + group_json_object(group) + "}";
}

std::string to_json_fix(const SolveFix& fix) {
  std::ostringstream os;
  os << '{';
  os << "\"icao\":";
  append_json_string(os, fix.icao);
  os << ",\"lat\":" << format_double(fix.lat);
  os << ",\"lon\":" << format_double(fix.lon);
  os << ",\"alt_ft\":" << format_double(fix.alt_ft);
  os << ",\"residual_m\":" << format_double(round_to(fix.residual_m, 2));
  os << ",\"quality_residual_m\":" << format_double(round_to(fix.quality_residual_m, 2));
  os << ",\"gdop\":" << format_double(round_to(fix.gdop, 2));
  os << ",\"num_sensors\":" << fix.num_sensors;
  os << ",\"solve_method\":";
  append_json_string(os, fix.solve_method);
  os << ",\"timestamp_s\":" << fix.timestamp_s;
  os << ",\"timestamp_ns\":" << fix.timestamp_ns;
  os << ",\"df_type\":" << fix.df_type;
  os << ",\"squawk\":" << squawk_json(fix.squawk);
  os << ",\"raw_msg\":";
  append_json_string(os, fix.raw_msg);
  os << ",\"t0_s\":" << format_double(round_to(fix.t0_s, 9));
  os << '}';
  return os.str();
}

std::string to_json_track(const TrackOutput& track) {
  std::ostringstream os;
  os << '{';
  os << "\"icao\":";
  append_json_string(os, track.icao);
  os << ",\"lat\":" << format_double(round_to(track.lat, 6));
  os << ",\"lon\":" << format_double(round_to(track.lon, 6));
  os << ",\"alt_ft\":" << format_double(round_to(track.alt_ft, 0));
  os << ",\"heading_deg\":" << format_double(round_to(track.heading_deg, 1));
  os << ",\"speed_kts\":" << format_double(round_to(track.speed_kts, 1));
  os << ",\"vrate_fpm\":" << format_double(round_to(track.vrate_fpm, 0));
  os << ",\"track_quality\":" << track.track_quality;
  os << ",\"positions_count\":" << track.positions_count;
  os << ",\"residual_m\":" << format_double(track.residual_m);
  os << ",\"quality_residual_m\":" << format_double(track.quality_residual_m);
  os << ",\"gdop\":" << format_double(track.gdop);
  os << ",\"num_sensors\":" << track.num_sensors;
  os << ",\"solve_method\":";
  append_json_string(os, track.solve_method);
  os << ",\"timestamp_s\":" << track.timestamp_s;
  os << ",\"timestamp_ns\":" << track.timestamp_ns;
  os << ",\"df_type\":" << track.df_type;
  os << ",\"squawk\":" << squawk_json(track.squawk);
  os << ",\"raw_msg\":";
  append_json_string(os, track.raw_msg);
  os << ",\"t0_s\":" << format_double(track.t0_s);
  os << ",\"cov_matrix\":[[" << format_double(track.cov_matrix[0][0]) << ',' << format_double(track.cov_matrix[0][1]) << "],[" << format_double(track.cov_matrix[1][0]) << ',' << format_double(track.cov_matrix[1][1]) << "]]";
  os << '}';
  return os.str();
}

std::string json_escape(std::string_view value) {
  std::string out;
  out.reserve(value.size());
  for (char c : value) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\b': out += "\\b"; break;
      case '\f': out += "\\f"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          std::ostringstream os;
          os << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(static_cast<unsigned char>(c));
          out += os.str();
        } else {
          out.push_back(c);
        }
        break;
    }
  }
  return out;
}

double round_to(double value, int digits) {
  double scale = std::pow(10.0, digits);
  return std::round(value * scale) / scale;
}

double clamp(double value, double lo, double hi) {
  return std::max(lo, std::min(value, hi));
}

}
