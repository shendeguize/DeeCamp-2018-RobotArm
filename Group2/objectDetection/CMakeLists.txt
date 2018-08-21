cmake_minimum_required(VERSION 3.1.0)

project(objectDetection)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS0} -g -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++0x")
endif()

# Package
find_package(OpenCV 3.1.0 REQUIRED)
find_package(realsense2 REQUIRED)

# Directory
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${realsense_INCLUDE_DIR}
)

# Link directory
link_directories()

# File
file(GLOB SOURCES
     "./source/*.cpp"
)
file(GLOB HEADERS
     "./header/*.h"
     "./header/*.hpp"
)

# Link library
set(PROJECT_LINK_LIBS
    ${OpenCV_LIBRARIES}
    ${realsense2_LIBRARY}
    ${DEPENDENCIES}
)

add_executable(objectDetection.out ${SOURCES})
target_link_libraries(objectDetection.out ${PROJECT_LINK_LIBS})