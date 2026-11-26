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
option(IMPULSE_BUILD_APP "Build the raylib demo app" ON)

include(FetchContent)

if(MSVC)
  set(IMPULSE_WARNINGS /W4)
else()
  set(IMPULSE_WARNINGS -Wall -Wextra -Wpedantic)
endif()

# all the physics lives here, no raylib allowed
add_library(impulse_core STATIC
  src/physics/Body.cpp
  src/physics/Collision.cpp
  src/physics/World.cpp
)
target_include_directories(impulse_core PUBLIC src)
target_compile_options(impulse_core PRIVATE ${IMPULSE_WARNINGS})

if(IMPULSE_BUILD_APP)
  # raylib is only used for the window, input and drawing
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(raylib)

  add_executable(impulse src/app/main.cpp)
  target_link_libraries(impulse PRIVATE impulse_core raylib)
  target_compile_options(impulse PRIVATE ${IMPULSE_WARNINGS})
endif()

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
