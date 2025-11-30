cmake_minimum_required(VERSION 3.16)
project(j1939_n2k)

if(NOT BUILD_EXAMPLES)
  find_package(isobus REQUIRED)
endif()
find_package(Threads REQUIRED)

add_executable(j1939_n2k main.cpp)

target_compile_features(j1939_n2k PUBLIC cxx_std_11)
set_target_properties(j1939_n2k PROPERTIES CXX_EXTENSIONS OFF)

target_link_libraries(
  j1939_n2k PRIVATE isobus::Isobus isobus::HardwareIntegration
                        Threads::Threads isobus::Utility)
