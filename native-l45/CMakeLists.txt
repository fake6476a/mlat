cmake_minimum_required(VERSION 3.20)
project(native_l45 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(CheckIPOSupported)

option(NATIVE_L45_PGO_GENERATE "Build with profile generation instrumentation" OFF)
option(NATIVE_L45_PGO_USE "Build with profile use optimization" OFF)
set(NATIVE_L45_PGO_PROFILE_DIR "${CMAKE_BINARY_DIR}/pgo-data" CACHE PATH "Directory for PGO profile data")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

check_ipo_supported(RESULT NATIVE_L45_IPO_SUPPORTED OUTPUT NATIVE_L45_IPO_OUTPUT)

if(NATIVE_L45_PGO_GENERATE AND NATIVE_L45_PGO_USE)
  message(FATAL_ERROR "NATIVE_L45_PGO_GENERATE and NATIVE_L45_PGO_USE cannot both be enabled")
endif()

set(NATIVE_L45_PGO_COMPILE_FLAGS)
set(NATIVE_L45_PGO_LINK_FLAGS)

if(NATIVE_L45_PGO_GENERATE)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    list(APPEND NATIVE_L45_PGO_COMPILE_FLAGS "-fprofile-generate=${NATIVE_L45_PGO_PROFILE_DIR}")
    list(APPEND NATIVE_L45_PGO_LINK_FLAGS "-fprofile-generate=${NATIVE_L45_PGO_PROFILE_DIR}")
  else()
    message(FATAL_ERROR "PGO generation is only configured for GCC and Clang")
  endif()
endif()

if(NATIVE_L45_PGO_USE)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    list(APPEND NATIVE_L45_PGO_COMPILE_FLAGS "-fprofile-use=${NATIVE_L45_PGO_PROFILE_DIR}")
    list(APPEND NATIVE_L45_PGO_COMPILE_FLAGS "-fprofile-correction")
    list(APPEND NATIVE_L45_PGO_COMPILE_FLAGS "-Wno-missing-profile")
    list(APPEND NATIVE_L45_PGO_LINK_FLAGS "-fprofile-use=${NATIVE_L45_PGO_PROFILE_DIR}")
    list(APPEND NATIVE_L45_PGO_LINK_FLAGS "-fprofile-correction")
  else()
    message(FATAL_ERROR "PGO use is only configured for GCC and Clang")
  endif()
endif()

add_library(native_l45_common
  src/json.cpp
  src/geo_adsb.cpp
  src/state.cpp
  src/solver.cpp
)

add_library(native_l45_layer4
  src/layer4.cpp
)

add_library(native_l45_layer5
  src/track.cpp
)

target_include_directories(native_l45_common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(native_l45_layer4 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(native_l45_layer5 PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_compile_options(native_l45_common PRIVATE
  -O3
  -march=native
  -Wall
  -Wextra
  -Wpedantic
  ${NATIVE_L45_PGO_COMPILE_FLAGS}
)

target_compile_options(native_l45_layer4 PRIVATE
  -O3
  -march=native
  -Wall
  -Wextra
  -Wpedantic
  ${NATIVE_L45_PGO_COMPILE_FLAGS}
)

target_compile_options(native_l45_layer5 PRIVATE
  -O3
  -march=native
  -Wall
  -Wextra
  -Wpedantic
  ${NATIVE_L45_PGO_COMPILE_FLAGS}
)

target_link_libraries(native_l45_layer4 PUBLIC native_l45_common)
target_link_libraries(native_l45_layer5 PUBLIC native_l45_common)

if(CMAKE_BUILD_TYPE STREQUAL "Release" AND NATIVE_L45_IPO_SUPPORTED)
  set_property(TARGET native_l45_common PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  set_property(TARGET native_l45_layer4 PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  set_property(TARGET native_l45_layer5 PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

add_executable(mlat-solver-native src/layer4_main.cpp)
add_executable(track-builder-native src/layer5_main.cpp)

target_compile_options(mlat-solver-native PRIVATE ${NATIVE_L45_PGO_COMPILE_FLAGS})
target_compile_options(track-builder-native PRIVATE ${NATIVE_L45_PGO_COMPILE_FLAGS})

target_link_libraries(mlat-solver-native PRIVATE native_l45_layer4)
target_link_libraries(track-builder-native PRIVATE native_l45_layer5)

target_link_options(mlat-solver-native PRIVATE ${NATIVE_L45_PGO_LINK_FLAGS})
target_link_options(track-builder-native PRIVATE ${NATIVE_L45_PGO_LINK_FLAGS})

if(CMAKE_BUILD_TYPE STREQUAL "Release" AND NATIVE_L45_IPO_SUPPORTED)
  set_property(TARGET mlat-solver-native PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
  set_property(TARGET track-builder-native PROPERTY INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()
