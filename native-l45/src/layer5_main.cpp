#include "core.hpp"

#include <exception>
#include <iostream>

int main() {
  std::ios::sync_with_stdio(false);
  std::cin.tie(nullptr);
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
