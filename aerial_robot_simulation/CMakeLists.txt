cmake_minimum_required(VERSION 3.0.2)
project(aerial_robot_simulation)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_base
  aerial_robot_estimation
  aerial_robot_msgs
  aerial_robot_model
  gazebo_ros_control
  kdl_parser
  roscpp
  spinal
  tf
  )

# MuJoCo is used only on simulation workstations.  Keep the Gazebo/real-machine
# workspace buildable when the optional mujoco_ros_pkgs workspace is absent.
find_package(mujoco_ros QUIET)
if(mujoco_ros_FOUND)
  find_package(mujoco_ros_control QUIET)
  find_package(mujoco_ros_sensors QUIET)
endif()

set(AERIAL_ROBOT_MUJOCO_AVAILABLE FALSE)
if(mujoco_ros_FOUND AND mujoco_ros_control_FOUND AND mujoco_ros_sensors_FOUND)
  set(AERIAL_ROBOT_MUJOCO_AVAILABLE TRUE)
  message(STATUS "Building aerial_robot_simulation MuJoCo plugins")
else()
  message(STATUS
    "MuJoCo packages are incomplete; building aerial_robot_simulation without MuJoCo plugins")
endif()

# Depend on system install of Gazebo
find_package(GAZEBO REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdfdom_headers REQUIRED)
include_directories(${orocos_kdl_INCLUDE_DIRS} ${urdfdom_headers_INCLUDE_DIRS})

set(AERIAL_ROBOT_SIMULATION_LIBRARIES
  aerial_robot_hw_sim
  spinal_interface
  flight_controllers
  rotor_controllers)
set(AERIAL_ROBOT_SIMULATION_CATKIN_DEPENDS
  aerial_robot_base aerial_robot_estimation aerial_robot_msgs
  aerial_robot_model gazebo_ros_control kdl_parser roscpp spinal tf)
if(AERIAL_ROBOT_MUJOCO_AVAILABLE)
  list(APPEND AERIAL_ROBOT_SIMULATION_LIBRARIES
    mujoco_aerial_robot_hw_sim mujoco_spinal_interface
    mujoco_thrust_visualizer)
  list(APPEND AERIAL_ROBOT_SIMULATION_CATKIN_DEPENDS
    mujoco_ros mujoco_ros_control mujoco_ros_sensors)
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${AERIAL_ROBOT_SIMULATION_LIBRARIES}
  CATKIN_DEPENDS ${AERIAL_ROBOT_SIMULATION_CATKIN_DEPENDS}
  DEPENDS GAZEBO orocos_kdl urdfdom_headers
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  include
  )

if(AERIAL_ROBOT_MUJOCO_AVAILABLE)
  include_directories(
    ${mujoco_ros_INCLUDE_DIRS}
    ${mujoco_ros_control_INCLUDE_DIRS}
    ${mujoco_ros_sensors_INCLUDE_DIRS})
endif()


add_definitions(-DSIMULATION)

set(FLIGHT_CONTROLLER_SOURCES src/simulation_attitude_controller.cpp)
if(AERIAL_ROBOT_MUJOCO_AVAILABLE)
  list(APPEND FLIGHT_CONTROLLER_SOURCES
    src/mujoco/mujoco_attitude_controller.cpp)
endif()
add_library(flight_controllers ${FLIGHT_CONTROLLER_SOURCES})
add_dependencies(flight_controllers spinal_generate_messages_cpp)
target_link_libraries(flight_controllers ${catkin_LIBRARIES})

add_library(rotor_controllers
  src/rotor_controller.cpp)
target_link_libraries(rotor_controllers ${catkin_LIBRARIES})

add_library(spinal_interface src/spinal_interface.cpp)
target_link_libraries(spinal_interface ${catkin_LIBRARIES})

add_library(aerial_robot_hw_sim src/aerial_robot_hw_sim.cpp)
target_link_libraries(aerial_robot_hw_sim spinal_interface ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${orocos_kdl_LIBRARIES})

if(AERIAL_ROBOT_MUJOCO_AVAILABLE)
  set(AERIAL_ROBOT_MUJOCO_LIBRARIES
    ${mujoco_ros_LIBRARIES}
    ${mujoco_ros_control_LIBRARIES}
    ${mujoco_ros_sensors_LIBRARIES})

  add_library(mujoco_spinal_interface src/mujoco/mujoco_spinal_interface.cpp)
  target_link_libraries(mujoco_spinal_interface
    ${catkin_LIBRARIES} ${AERIAL_ROBOT_MUJOCO_LIBRARIES})

  add_library(mujoco_aerial_robot_hw_sim
    src/mujoco/mujoco_aerial_robot_hw_sim.cpp
    src/mujoco/mujoco_default_aerial_robot_hw_sim.cpp)
  target_link_libraries(mujoco_aerial_robot_hw_sim
    ${catkin_LIBRARIES} ${AERIAL_ROBOT_MUJOCO_LIBRARIES}
    mujoco_spinal_interface)

  add_library(mujoco_thrust_visualizer src/mujoco/thrust_visualizer_plugin.cpp)
  target_link_libraries(mujoco_thrust_visualizer
    ${catkin_LIBRARIES} ${AERIAL_ROBOT_MUJOCO_LIBRARIES})
endif()


install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS ${AERIAL_ROBOT_SIMULATION_LIBRARIES}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY scripts plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
