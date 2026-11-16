cmake_minimum_required(VERSION 3.20)
project(warehouse_world LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

add_library(ww_core
  src/grid.cpp
  src/systems.cpp
  src/simulation.cpp
  src/render.cpp
)
target_include_directories(ww_core PUBLIC include)
target_compile_options(ww_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(warehouse apps/main.cpp)
target_link_libraries(warehouse PRIVATE ww_core)
target_compile_options(warehouse PRIVATE -Wall -Wextra -Wpedantic)

enable_testing()
add_executable(ww_tests
  tests/test_main.cpp
  tests/test_grid.cpp
  tests/test_ecs.cpp
  tests/test_timestep.cpp
  tests/test_simulation.cpp
)
target_link_libraries(ww_tests PRIVATE ww_core)
target_include_directories(ww_tests PRIVATE tests)
add_test(NAME ww_tests COMMAND ww_tests)
