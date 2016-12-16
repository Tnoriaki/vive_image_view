cmake_minimum_required(VERSION 2.8.3)
project(vive_image_view)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  std_srvs
  geometry_msgs
	roslib
	camera_calibration_parsers#↓image_view
	cv_bridge dynamic_reconfigure
	image_transport
	message_filters
	message_generation
	nodelet
	rosconsole
)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")

catkin_package()


#############  assume openvr is installed in home directory  ################
set(OpenVR_SOURCE_DIR "$ENV{HOME}/openvr")

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${OpenVR_SOURCE_DIR}/cmake")
set(ARCH "64" CACHE STRING
  "ARCH chosen by the user at CMake configure time")
set_property(CACHE ARCH PROPERTY STRINGS 32 64)

message(STATUS "Compilation for '${ARCH}' Bit")


message(STATUS "OpenVR_SOURCE_DIR '${OpenVR_SOURCE_DIR}' ")
message(STATUS "OPENVR '${OPENVR}' ")
message(STATUS "SDL2_INCLUDE_DIR '${SDL2_INCLUDE_DIR}' ")


#image_view
find_package(Boost REQUIRED COMPONENTS signals thread)
find_package(OpenCV REQUIRED)

include_directories(${Boost_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS}
                    ${OpenCV_INCLUDE_DIRS}
)

find_package(GTK2)
add_definitions(-DHAVE_GTK)
include_directories(${GTK2_INCLUDE_DIRS})

find_package(SDL2 REQUIRED)

include_directories(${SDL2_INCLUDE_DIR} "${OpenVR_SOURCE_DIR}/headers" "${OpenVR_SOURCE_DIR}/samples")

find_library(OPENVR_LIB openvr_api "${OpenVR_SOURCE_DIR}/lib/linux${ARCH}" NO_DEFAULT_PATH) #TODO other operating systems
MESSAGE ("Found libopenvr_api: ${OPENVR_LIB}")

find_package(GLEW REQUIRED)

if ("${ARCH}" STREQUAL "32")
        set_target_properties(vive_image_view PROPERTIES COMPILE_FLAGS "-m32" LINK_FLAGS "-m32")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 ")

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenVR_SOURCE_DIR}/headers
  ${GLEW_INCLUDE_DIRS}
)

add_executable(vive_image_view_node
  vive_image_view_node.cpp
  ${OpenVR_SOURCE_DIR}/samples/shared/pathtools.cpp
	${OpenVR_SOURCE_DIR}/samples/shared/lodepng.cpp
	${OpenVR_SOURCE_DIR}/samples/shared/Matrices.cpp
)

target_link_libraries (vive_image_view_node LINK_PUBLIC
	openvr_api
	${SDL2_LIBRARY}
	${GLEW_INCLUDE_DIRS}
	${catkin_LIBRARIES}
	${OPENVR_LIB}
	${GLEW_LIBRARIES}
	dl
	GL
	GLU
	${GTK_LIBRARIES}#↓image_view
	${GTK2_LIBRARIES}
	${OpenCV_LIBRARIES}
	${Boost_LIBRARIES}
)



