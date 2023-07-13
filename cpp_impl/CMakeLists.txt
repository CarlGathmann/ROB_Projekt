cmake_minimum_required(VERSION 3.10)
project(ROB_Projekt)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED)
set(EIGEN_INCLUDE_DIR "C:/Program Files (x86)/Eigen3/include/eigen3")
message("${EIGEN_INCLUDE_DIR}")
message(${Eigen3_FOUND})
include_directories(${EIGEN_INCLUDE_DIR})
add_executable(${PROJECT_NAME} triangle_cpp.cpp)

target_link_libraries(${PROJECT_NAME}
        ${Eigen_LIBRARIES}
        )
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_directory
        C:/Users/FredvomJupiter/CLionProjects/ROB_Projekt/meshes ${CMAKE_BINARY_DIR}/meshes)