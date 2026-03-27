cmake_minimum_required(VERSION 3.0.2)
project(nexus_swarm_sim)

add_compile_options(-std=c++11)

find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(gazebo REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_ros
  geometry_msgs
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  UwbRange.msg
  RawUWBSignal.msg
)

add_service_files(
  FILES
  CheckLineOfSight.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    gazebo_msgs
    gazebo_ros
    geometry_msgs
    message_runtime
    roscpp
    rospy
    sensor_msgs
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${SDFormat_INCLUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_uwb_simulator
  src/uwb_simulator.cpp
)

add_library(
  ${PROJECT_NAME}_los_raycast_plugin
  src/los_raycast_plugin.cpp
)

add_dependencies(
  ${PROJECT_NAME}_uwb_simulator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  ${PROJECT_NAME}_los_raycast_plugin
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

set_target_properties(
  ${PROJECT_NAME}_uwb_simulator
  PROPERTIES OUTPUT_NAME uwb_simulator PREFIX ""
)

target_link_libraries(
  ${PROJECT_NAME}_uwb_simulator
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

target_link_libraries(
  ${PROJECT_NAME}_los_raycast_plugin
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/start_sitl_session.py
  scripts/launch_mavros_bridge.py
  scripts/configure_stream_rates.py
  scripts/spawn_ardupilot_model.py
  scripts/swarm_dashboard.py
  src/swarm_uwb_monitor.py
  scripts/swarm_launcher.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  models
  scripts/dashboard
  worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts/dashboard/
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/dashboard
  FILES_MATCHING PATTERN "*.py"
)
