cmake_minimum_required(VERSION 3.16)
project(ir_lasertag_host_tests C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

# Stub ESP-IDF headers
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/stubs
  ${CMAKE_CURRENT_SOURCE_DIR}/../include
)

add_executable(test_ir_protocol
  test_protocol_config.cpp
  test_arclite_tagcode.cpp
  test_main.cpp
  # Source file under test (pure config logic, no hardware deps)
  ${CMAKE_CURRENT_SOURCE_DIR}/../src/codec/ir_protocol_config.cpp
)

add_test(NAME ir_protocol_tests COMMAND test_ir_protocol)
