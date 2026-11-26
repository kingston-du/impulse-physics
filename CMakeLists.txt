cmake_minimum_required(VERSION 3.20)
project(impulse LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(IMPULSE_BUILD_TESTS "Build unit tests" ON)

include(FetchContent)

# header only for now, the physics sources get added later
add_library(impulse_core INTERFACE)
target_include_directories(impulse_core INTERFACE src)

if(MSVC)
  set(IMPULSE_WARNINGS /W4)
else()
  set(IMPULSE_WARNINGS -Wall -Wextra -Wpedantic)
endif()

add_executable(impulse src/app/main.cpp)
target_link_libraries(impulse PRIVATE impulse_core)
target_compile_options(impulse PRIVATE ${IMPULSE_WARNINGS})

if(IMPULSE_BUILD_TESTS)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.7.1
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(Catch2)

  enable_testing()
  add_executable(impulse_tests tests/test_vec2.cpp)
  target_link_libraries(impulse_tests PRIVATE impulse_core Catch2::Catch2WithMain)
  target_compile_options(impulse_tests PRIVATE ${IMPULSE_WARNINGS})

  list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
  include(Catch)
  catch_discover_tests(impulse_tests)
endif()
