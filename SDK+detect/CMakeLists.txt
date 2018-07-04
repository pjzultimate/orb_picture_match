cmake_minimum_required(VERSION 2.8)

project(camtest2)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

find_package(OpenCV REQUIRED)

include_directories("${PROJECT_SOURCE_DIR}/src")
link_directories("${PROJECT_SOURCE_DIR}/src")

add_executable(camtest2 src/camtest2.cpp)

target_link_libraries(camtest2 loitorusbcam loitorimu usb-1.0 ${OpenCV_LIBS})
