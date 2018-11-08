cmake_minimum_required(VERSION 3.12)
project(raycasting)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(raycasting main.cpp)
target_link_libraries(raycasting ${OpenCV_LIBS})
