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
  src/astar.cpp
  src/reservation.cpp
  src/stastar.cpp
  src/cbs.cpp
  src/simulation.cpp
  src/render.cpp
)
target_include_directories(ww_core PUBLIC include)
target_compile_options(ww_core PRIVATE -Wall -Wextra -Wpedantic)

add_executable(warehouse apps/main.cpp)
target_link_libraries(warehouse PRIVATE ww_core)
target_compile_options(warehouse PRIVATE -Wall -Wextra -Wpedantic)

# Graphical viewer: opt-in so the default build needs no downloads.
# Given: raylib from FetchContent, pinned. Approach: build it static next to
# the viewer; under Emscripten build raylib for the web and emit viewer.html.
option(WW_VIEWER "Build the raylib graphical viewer (downloads raylib)" OFF)
if(WW_VIEWER)
  include(FetchContent)
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  if(EMSCRIPTEN)
    set(PLATFORM Web CACHE STRING "" FORCE)
  endif()
  FetchContent_Declare(raylib
    URL https://github.com/raysan5/raylib/archive/refs/tags/5.5.tar.gz
    URL_HASH SHA256=aea98ecf5bc5c5e0b789a76de0083a21a70457050ea4cc2aec7566935f5e258e
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_MakeAvailable(raylib)

  add_executable(viewer apps/viewer.cpp)
  target_link_libraries(viewer PRIVATE ww_core raylib)
  target_compile_options(viewer PRIVATE -Wall -Wextra)
  if(EMSCRIPTEN)
    set_target_properties(viewer PROPERTIES SUFFIX ".html")
    target_link_options(viewer PRIVATE -sUSE_GLFW=3 -sALLOW_MEMORY_GROWTH=1
      --shell-file ${CMAKE_SOURCE_DIR}/apps/viewer_shell.html)
  endif()
endif()

if(EMSCRIPTEN)
  return()  # tests and the terminal app are native-only
endif()

enable_testing()
add_executable(ww_tests
  tests/test_main.cpp
  tests/test_grid.cpp
  tests/test_ecs.cpp
  tests/test_timestep.cpp
  tests/test_simulation.cpp
  tests/test_random.cpp
  tests/test_astar.cpp
  tests/test_stastar.cpp
  tests/test_cbs.cpp
)
target_link_libraries(ww_tests PRIVATE ww_core)
target_include_directories(ww_tests PRIVATE tests)
add_test(NAME ww_tests COMMAND ww_tests)
