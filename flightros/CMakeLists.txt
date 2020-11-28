cmake_minimum_required(VERSION 3.0.0)
project(flightros)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    rospy
    roscpp

    flightlib
    autopilot
    quadrotor_common
    pcl_ros

    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    message_generation
    image_transport
    cv_bridge
    sensor_msgs
    tf_conversions
    tf
)

find_package(OpenCV REQUIRED)
# find_package( PCL REQUIRED)

# catkin_simple()

# Declare the service files to be built
add_service_files(FILES QuadState.srv)

# These also go here:
# add_message_files()
# add_action_files()

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs sensor_msgs geometry_msgs
  LIBRARIES flight_pilot depth_converter
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)










# Setup Default Build Type as Release
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "" FORCE)
endif ()

# Setup Architcture-specific Flags
if ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "armv7l")
    message(STATUS "Using ARMv7 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv7-a -mfpu=neon -mfloat-abi=hard -funsafe-math-optimizations")
elseif ("${CMAKE_HOST_SYSTEM_PROCESSOR}" STREQUAL "aarch64")
    message(STATUS "Using ARM aarch64 optimized flags!")
    set(CMAKE_CXX_ARCH_FLAGS " -Wno-psabi -march=armv8-a+crypto -mcpu=cortex-a57+crypto")
else ()
    set(CMAKE_CXX_ARCH_FLAGS " -march=native")
endif ()

# Setup General C++ Flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DEIGEN_STACK_ALLOCATION_LIMIT=1048576")

# Setup Release and Debug flags
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} ${CMAKE_CXX_ARCH_FLAGS} -Wall -DNDEBUG -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wall -g")











# PCL Stuff
include_directories(${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(flight_pilot src/flight_pilot.cpp)
add_library(depth_converter src/depth_converter.cpp)

target_link_libraries(flight_pilot
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  stdc++fs
)
target_link_libraries(depth_converter
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  stdc++fs
)

add_executable(flight_pilot_node src/flight_pilot_node.cpp)
add_executable(depth_converter_node src/depth_converter_node.cpp)

target_link_libraries(flight_pilot_node
  flight_pilot
  ${OpenCV_LIBRARIES}
  stdc++fs
)
target_link_libraries(depth_converter_node
  depth_converter 
  ${OpenCV_LIBRARIES}
  stdc++fs
)

add_executable(navigator_node src/navigator.cpp)
target_link_libraries(navigator_node
  ${catkin_LIBRARIES}
)

# Finish
# cs_install()
# cs_export()

install(TARGETS flight_pilot
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(TARGETS depth_converter
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
