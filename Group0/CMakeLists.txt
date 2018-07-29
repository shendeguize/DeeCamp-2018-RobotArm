cmake_minimum_required(VERSION 2.8.7)
project(sample)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -fexceptions -frtti -pthread -O3 -march=core2")

find_package(OpenCV 3 REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
message("Found Eigen3 in: ${EIGEN3_INCLUDE_DIR}")

set(ROOT 		"${CMAKE_CURRENT_SOURCE_DIR}/")
set(LIB_REALSENSE 	"/usr/lib/x86_64-linux-gnu/librealsense2.so")

include_directories(
    ${ROOT}/
    ${OpenCV_INCLUDE_DIRS}
)

link_directories(
    ${ROOT}/lib
)

file(GLOB SOURCES
    "*.cpp"
    )

set( PROJECT_LINK_LIBS
    ${LIB_REALSENSE}
    ${OpenCV_LIBRARIES}
)
add_executable(sample ${SOURCES})
target_link_libraries(sample ${PROJECT_LINK_LIBS})

add_library(realsensedetection SHARED ${SOURCES})
target_link_libraries(realsensedetection ${PROJECT_LINK_LIBS})
