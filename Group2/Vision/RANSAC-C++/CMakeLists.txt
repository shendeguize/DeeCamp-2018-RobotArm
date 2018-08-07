cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ransac_plane)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

#RealSense
find_package( realsense2 REQUIRED )
include_directories( ${realsense_INCLUDE_DIR} )

#OpenCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(ransac_plane ransac_plane.cpp)

target_link_libraries(ransac_plane ${PCL_LIBRARIES} ${OpenCV_LIBS} ${realsense2_LIBRARY} ${DEPENDENCIES})

###set(CMAKE_BUILD_TYPE Debug) 
#set(DEPENDENCIES realsense2 ${PCL_LIBRARIES} ${OpenCV_LIBS})
