set(OpenCV_DIR "../OCV_install")
set(CMAKE_BUILD_FILES_DIRECTORY "./build")

cmake_minimum_required(VERSION 2.8)
project( fisheye_stereo )

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( dewrapper dewrapper.cpp )
target_link_libraries( dewrapper ${OpenCV_LIBS} )