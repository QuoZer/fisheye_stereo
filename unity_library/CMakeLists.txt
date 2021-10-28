set(OpenCV_DIR "../../../OCV_install")
set(CMAKE_BUILD_FILES_DIRECTORY "./build")

cmake_minimum_required(VERSION 2.8)

project( Unity_plugin)

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable( unity_plugin unity_plugin.cpp )
target_link_libraries( unity_plugin ${OpenCV_LIBS} )