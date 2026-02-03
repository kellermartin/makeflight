cmake_minimum_required(VERSION 3.20)

project(makeflight LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(flightcore
  src/actuators/biheli_pwm_output.cpp
  src/actuators/dshot_output.cpp
  src/actuators/pwm_output.cpp
  src/config/in_memory_config.cpp
  src/controllers/basic_controller.cpp
  src/controllers/rov_controller.cpp
  src/estimators/madgwick.cpp
  src/hal/rp2350_hal.cpp
  src/receiver/udp_receiver.cpp
  src/scheduler/scheduler.cpp
  src/sensors/mpu6050.cpp
  src/vehicle/vehicle.cpp
  src/vehicle/rov4_vehicle.cpp
)

target_include_directories(flightcore PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_options(flightcore PRIVATE -Wall -Wextra -Wpedantic)

option(BUILD_DEMO "Build demo executable" ON)
if (BUILD_DEMO)
  add_executable(flight_demo src/main.cpp)
  target_link_libraries(flight_demo PRIVATE flightcore)
endif()

option(BUILD_TESTS "Build unit tests" ON)
if (BUILD_TESTS)
  include(FetchContent)
  FetchContent_Declare(
    doctest
    GIT_REPOSITORY https://github.com/doctest/doctest.git
    GIT_TAG v2.4.11
  )
  FetchContent_MakeAvailable(doctest)

  enable_testing()

  add_executable(flight_tests
    tests/test_main.cpp
    tests/test_scheduler.cpp
    tests/test_rov_controller.cpp
    tests/test_biheli_pwm.cpp
    tests/test_config.cpp
    tests/test_mpu6050.cpp
    tests/test_udp_receiver.cpp
  )
  target_link_libraries(flight_tests PRIVATE flightcore doctest::doctest)

  option(ENABLE_COVERAGE "Enable coverage reporting" OFF)
  if (ENABLE_COVERAGE)
    if (CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
      target_compile_options(flight_tests PRIVATE -O0 -g --coverage)
      target_link_options(flight_tests PRIVATE --coverage)
    endif()
  endif()

  add_test(NAME flight_tests COMMAND flight_tests)
endif()

find_package(Doxygen)
if (DOXYGEN_FOUND)
  add_custom_target(docs
    COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM
  )
endif()

find_program(MKDOCS_EXECUTABLE mkdocs)
if (MKDOCS_EXECUTABLE)
  add_custom_target(mkdocs
    COMMAND ${MKDOCS_EXECUTABLE} build
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building MkDocs site"
    VERBATIM
  )
endif()
