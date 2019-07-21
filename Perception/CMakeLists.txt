cmake_minimum_required(VERSION 2.8)
project( Perception)
find_package( OpenCV REQUIRED )
find_package(PkgConfig)

pkg_check_modules(GST REQUIRED gstreamer-1.0>=1.4
                               gstreamer-sdp-1.0>=1.4
                               gstreamer-video-1.0>=1.4
                               gstreamer-app-1.0>=1.4) 


message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS} ")

 message( "found = ${GST_FOUND}" )
    message( "dirs = ${GST_INCLUDE_DIRS}" )
    message( "libs = ${GST_LIBRARIES}" )

if(CMAKE_VERSION VERSION_LESS "2.8.11")
    # Add OpenCV headers location to your include paths
    include_directories(${OpenCV_INCLUDE_DIRS})
endif()

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
include_directories (${GST_INCLUDE_DIRS} )

add_executable( Perception Perception.cpp )
target_link_libraries( Perception ${OpenCV_LIBS} ${GST_LIBRARIES} )

