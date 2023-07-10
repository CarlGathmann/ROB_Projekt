cmake_minimum_required(VERSION 3.10)
project(ROB_Projekt)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(ROB_Projekt triangle_cpp.cpp triangle_point_dist.cpp)

target_include_directories(ROB_Projekt
        PRIVATE
        "${PROJECT_SOURCE_DIR/eigen-3.4.0}"
        "${PROJECT_SOURCE_DIR/libigl-main/include}"
        )

