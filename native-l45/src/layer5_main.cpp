// layer5_main.cpp — Entry point for the Layer 5 track builder binary.
// Reads JSONL L4 records from stdin, writes EKF-filtered track updates to stdout.
#include "core.hpp"

#include <exception>
#include <iostream>

int main() {
  std::ios::sync_with_stdio(false);
  std::cin.tie(nullptr);
  static char out_buf[1 << 16];
  std::cout.rdbuf()->pubsetbuf(out_buf, sizeof(out_buf));
  try {
    return native_l45::run_layer5(std::cin, std::cout, std::cerr);
  } catch (const std::exception& ex) {
    std::cerr << ex.what() << '\n';
    return 1;
  } catch (...) {
    std::cerr << "unknown error\n";
    return 1;
  }
}
