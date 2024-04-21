PROJECT(ut_turtlebots)
CMAKE_MINIMUM_REQUIRED(VERSION 3.8)

message(STATUS "Compiling with ROS2 COLCON")
find_package(ament_cmake REQUIRED)


# MESSAGE(STATUS "Compilers found: ${CMAKE_CXX_COMPILER_LIST}")
# MESSAGE(STATUS "Using compiler: ${CMAKE_CXX_COMPILER}")
# MESSAGE(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
# MESSAGE(STATUS "Build Mode: ${CMAKE_BUILD_MODE}")
# MESSAGE(STATUS "Arch: ${CMAKE_SYSTEM_PROCESSOR}")

# SET(CMAKE_AUTOMOC ON)
# SET(CMAKE_AUTORCC ON)
# SET(CMAKE_AUTOUIC ON)

# IF(CMAKE_VERSION VERSION_LESS "3.7.0")
#     SET(CMAKE_INCLUDE_CURRENT_DIR ON)
# ENDIF()

SET(CMAKE_CXX_FLAGS "-std=c++11 -march=native -Werror -Wall -g")

IF(${CMAKE_BUILD_TYPE} MATCHES "Release")
  MESSAGE(STATUS "Additional Flags for Release mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -fopenmp -O3 -DNDEBUG")
ELSEIF(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  MESSAGE(STATUS "Additional Flags for Debug mode")
  SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g")
  SET(BUILD_SPECIFIC_LIBRARIES "")
ENDIF()

FIND_PACKAGE(Qt5 COMPONENTS Core Widgets Gui WebSockets OpenGL REQUIRED)
FIND_PACKAGE(geometry_msgs REQUIRED)
FIND_PACKAGE(amrl_msgs REQUIRED)
FIND_PACKAGE(rclcpp REQUIRED)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

MESSAGE(STATUS "ROS-Overrride Build Type: ${CMAKE_BUILD_TYPE}")
MESSAGE(STATUS "CXX Flags: ${CMAKE_CXX_FLAGS}")
MESSAGE(STATUS "Build-Specific Libraries: ${BUILD_SPECIFIC_LIBRARIES}")

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/submodules")

INCLUDE_DIRECTORIES(${PROJECT_BINARY_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

MESSAGE(STATUS "Adding subdirectory")
ADD_SUBDIRECTORY(src/shared)
MESSAGE(STATUS "Done adding subdirectory")
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src/shared ${geometry_msgs_INCLUDE_DIRS} ${amrl_msgs_INCLUDE_DIRS})

SET(libs rclcpp glog gflags amrl_shared_lib geometry_msgs
    ${BUILD_SPECIFIC_LIBRARIES} rosbag X11 lua5.1 boost_system)

ADD_EXECUTABLE(websocket
    src/websocket/websocket_main.cc
    src/websocket/websocket.cc
)
TARGET_LINK_LIBRARIES(websocket
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::WebSockets
    ${libs})

ament_target_dependencies(websocket geometry_msgs amrl_msgs rclcpp)

ament_package()
