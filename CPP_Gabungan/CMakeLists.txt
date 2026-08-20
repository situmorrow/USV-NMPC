cmake_minimum_required(VERSION 3.16)
project(usv_nmpc_cpp_gabungan LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optimization flags for real-time SQP solver performance
if(MSVC)
  add_compile_options(/O2 /W3 /openmp)
else()
  add_compile_options(-O3 -Wall -Wextra -Wpedantic -fopenmp)
  add_link_options(-fopenmp)
endif()

# -----------------------------------------------------------------------------
# 1. NMPC Codegen Solver Static Library
# -----------------------------------------------------------------------------
file(GLOB NMPC_SOURCES 
  "nmpc/*.cpp"
)

# Exclude any main/example files if matched
list(FILTER NMPC_SOURCES EXCLUDE REGEX ".*examples.*")
list(FILTER NMPC_SOURCES EXCLUDE REGEX ".*interface.*")

add_library(nmpc_solver STATIC ${NMPC_SOURCES})
target_include_directories(nmpc_solver PUBLIC nmpc)

# -----------------------------------------------------------------------------
# 2. Guidance & State Observer Libraries
# -----------------------------------------------------------------------------
add_library(guidance_lib STATIC src/guidance.cpp)
target_include_directories(guidance_lib PUBLIC include)

add_library(state_observer_lib STATIC src/state_observer.cpp)
target_include_directories(state_observer_lib PUBLIC include)

# -----------------------------------------------------------------------------
# 3. Unit Test Executables
# -----------------------------------------------------------------------------
# Test 1: NMPC Solver standalone benchmark
add_executable(test_nmpc tests/test_nmpc.cpp)
target_link_libraries(test_nmpc PRIVATE nmpc_solver)
target_include_directories(test_nmpc PRIVATE include nmpc)

# Test 2: Guidance & ENU Coordinate Transformation test
add_executable(test_guidance tests/test_guidance.cpp)
target_link_libraries(test_guidance PRIVATE guidance_lib)
target_include_directories(test_guidance PRIVATE include)

# Test 3: State Observer & IMU Synchronization test
add_executable(test_state_observer tests/test_state_observer.cpp)
target_link_libraries(test_state_observer PRIVATE state_observer_lib guidance_lib nmpc_solver)
target_include_directories(test_state_observer PRIVATE include nmpc)

# Test 4: Standalone Closed-Loop Trajectory Simulation
add_executable(test_nmpc_sim tests/test_nmpc_sim.cpp)
target_link_libraries(test_nmpc_sim PRIVATE guidance_lib state_observer_lib nmpc_solver)
target_include_directories(test_nmpc_sim PRIVATE include nmpc)

# -----------------------------------------------------------------------------
# 4. Main Serial & Telemetry Control Executable
# -----------------------------------------------------------------------------
add_executable(read_write_serial
  src/main.cpp
  src/serial_port.cpp
  src/guidance.cpp
  src/state_observer.cpp
)
target_link_libraries(read_write_serial PRIVATE nmpc_solver guidance_lib state_observer_lib)
target_include_directories(read_write_serial PRIVATE include nmpc)
