cmake_minimum_required(VERSION 3.10)
project(push_tracker)

set(CMAKE_CXX_STANDARD 17)

# --- Optimization flags ---
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -mfma -mavx -mavx2 -mf16c -ffast-math")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native -mfma -mavx -mavx2 -mf16c -ffast-math")

# --- Raylib ---
add_subdirectory(external/raylib)

# --- System OpenCV ---
find_package(OpenCV REQUIRED)

# --- OpenGL ---
find_package(OpenGL REQUIRED)

# --- Realsense2 ---
find_package(realsense2 REQUIRED)

# --- Include directories ---
include_directories(
    external/raylib/src
    ${OpenCV_INCLUDE_DIRS}
    ${realsense2_INCLUDE_DIRS}
)

# --- Game executable ---
add_executable(push_tracker
    src/main.cpp
    src/Tracker.cpp
)

# --- Link everything together ---
target_link_libraries(push_tracker
    raylib
    ${OpenCV_LIBS}
    ${OpenGL_LIBRARIES}
    GL
    realsense2
)
