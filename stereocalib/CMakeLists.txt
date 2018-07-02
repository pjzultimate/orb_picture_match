cmake_minimum_required(VERSION 3.10)
project(untitled2)

set(CMAKE_CXX_STANDARD 11)
find_package(OpenCV 3.3.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(untitled2 main.cpp parameters.h parameters.cpp stereo.cpp stereo.h)
target_link_libraries(untitled2 ${OpenCV_LIBS})