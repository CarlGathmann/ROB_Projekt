cmake_minimum_required(VERSION 3.10)
project(ROB_Projekt)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

add_executable(ROB_Projekt triangle_cpp.cpp)

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        C:/Users/FredvomJupiter/CLionProjects/ROB_Projekt/meshes ${CMAKE_BINARY_DIR}/meshes)