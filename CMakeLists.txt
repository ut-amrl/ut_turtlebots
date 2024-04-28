cmake_minimum_required(VERSION 3.5)
project(ut_turtlebots)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)


SET(CMAKE_AUTOMOC ON)
SET(CMAKE_AUTORCC ON)
SET(CMAKE_AUTOUIC ON)

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/submodules")


find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(amrl_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Qt5 COMPONENTS Core Widgets Gui WebSockets OpenGL REQUIRED)
SET(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(src/shared)
ADD_SUBDIRECTORY(src/shared)
include_directories(include)
include_directories(${Qt5_INCLUDE_DIRS})
include_directories(src)

SET(libs glog gflags amrl_shared_lib lua5.1 boost_system)

# add_library(websocket_lib SHARED
            # src/websocket/websocket.cpp
            # src/websocket/publisher_node.cpp
            # src/websocket/subscriber_node.cpp)
# target_compile_definitions(websocket_lib
  # PRIVATE "MINIMAL_COMPOSITION_DLL")
# target_link_libraries(websocket_lib Qt5::Core Qt5::Gui Qt5::Widgets Qt5::WebSockets)
# ament_target_dependencies(websocket_lib rclcpp rclcpp_components std_msgs amrl_msgs ${Qt5_LIBRARIES})

# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif()

add_executable(websocket_main src/websocket/websocket_main.cpp src/websocket/websocket.cpp include/websocket/websocket.hpp)
target_link_libraries(websocket_main 
  # websocket_lib 
  amrl_shared_lib
  gflags
  glog
  Qt5::Core 
  Qt5::Gui
  Qt5::Widgets
  Qt5::WebSockets
  ${Qt5_LIBRARIES})
ament_target_dependencies(websocket_main rclcpp amrl_msgs sensor_msgs class_loader)


add_executable(simulator
        src/simulator/simulator.cc
        src/simulator/vector_map.cc
        src/simulator/simulator_main.cc)
target_link_libraries(simulator ${libs})
ament_target_dependencies(simulator rclcpp amrl_msgs sensor_msgs tf2 tf2_ros geometry_msgs visualization_msgs)

add_executable(step_simulator
        src/simulator/simulator.cc
        src/simulator/vector_map.cc
        src/simulator/step_simulator_main.cc)
target_link_libraries(step_simulator ${libs})
ament_target_dependencies(step_simulator rclcpp amrl_msgs sensor_msgs tf2 tf2_ros geometry_msgs visualization_msgs)


# install(TARGETS
#   websocket_lib
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)

install(TARGETS
  websocket_main
  step_simulator
  simulator
  DESTINATION lib/${PROJECT_NAME})

INSTALL(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
