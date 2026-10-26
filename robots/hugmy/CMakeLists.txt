cmake_minimum_required(VERSION 3.0.2)
project(hugmy)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  aerial_robot_control
  aerial_robot_model
  aerial_robot_simulation
  pluginlib
  std_msgs
  std_srvs
  spinal
  geometry_msgs
  nav_msgs
  sensor_msgs
  jsk_recognition_msgs
  aerial_robot_msgs
  cv_bridge
  tf
  image_transport
  visualization_msgs
  )

find_package(PythonInterp REQUIRED)
find_package(mujoco_ros QUIET)
if(mujoco_ros_FOUND)
  find_package(mujoco_ros_control QUIET)
endif()
find_package(mujoco_ros_utils QUIET)

list(FIND aerial_robot_simulation_LIBRARIES
  mujoco_aerial_robot_hw_sim HUGMY_MUJOCO_BASE_LIBRARY_INDEX)
set(HUGMY_MUJOCO_AVAILABLE FALSE)
if(mujoco_ros_FOUND AND mujoco_ros_control_FOUND AND mujoco_ros_utils_FOUND
   AND NOT HUGMY_MUJOCO_BASE_LIBRARY_INDEX EQUAL -1)
  set(HUGMY_MUJOCO_AVAILABLE TRUE)
  message(STATUS "Building Hugmy MuJoCo hardware plugin and models")
else()
  message(STATUS
    "MuJoCo packages are incomplete; building Hugmy real-machine control only")
endif()

set(HUGMY_EXPORTED_LIBRARIES hugmy_quad_bi_controller)
set(HUGMY_CATKIN_DEPENDS
  aerial_robot_control pluginlib aerial_robot_model aerial_robot_simulation
  spinal std_msgs std_srvs geometry_msgs sensor_msgs tf visualization_msgs)
if(HUGMY_MUJOCO_AVAILABLE)
  list(APPEND HUGMY_EXPORTED_LIBRARIES hugmy_pneumatic_mujoco_hw_sim)
  list(APPEND HUGMY_CATKIN_DEPENDS mujoco_ros_control)
endif()

catkin_package(
  LIBRARIES ${HUGMY_EXPORTED_LIBRARIES}
  CATKIN_DEPENDS ${HUGMY_CATKIN_DEPENDS}
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

if(HUGMY_MUJOCO_AVAILABLE)
  include_directories(${mujoco_ros_control_INCLUDE_DIRS})
  add_library(hugmy_pneumatic_mujoco_hw_sim
    src/mujoco/hugmy_pneumatic_hw_sim.cpp)
  add_dependencies(hugmy_pneumatic_mujoco_hw_sim ${catkin_EXPORTED_TARGETS})
  target_compile_definitions(hugmy_pneumatic_mujoco_hw_sim PRIVATE SIMULATION)
  # HugmyPneumaticHWSim derives from AerialRobotHWSim.  Keep the base plugin
  # as an explicit ELF dependency: relying only on pluginlib load order leaves
  # its virtual methods/RTTI unresolved when this derived plugin loads first.
  target_link_libraries(hugmy_pneumatic_mujoco_hw_sim
    ${catkin_LIBRARIES} ${mujoco_ros_control_LIBRARIES})
endif()

add_library(hugmy_navigation src/navigation/hugmy_navigation.cpp)
target_link_libraries(hugmy_navigation ${catkin_LIBRARIES})

add_library(hugmy_deformation_planning src/control/deformation_planning.cpp)
target_link_libraries(hugmy_deformation_planning ${catkin_LIBRARIES})
add_dependencies(hugmy_deformation_planning ${catkin_EXPORTED_TARGETS})

add_executable(hugmy_approaching_planning src/control/approaching_with_interaction.cpp)
target_link_libraries(hugmy_approaching_planning ${catkin_LIBRARIES})
add_dependencies(hugmy_approaching_planning ${catkin_EXPORTED_TARGETS})

add_library(hugmy_morphing_arm_controller src/control/morphing_arm_controller.cpp)
target_link_libraries(hugmy_morphing_arm_controller ${catkin_LIBRARIES})

add_library(lpv_lqi_qp_controller src/control/lpv_lqi_qp_controller.cpp)
target_link_libraries(lpv_lqi_qp_controller ${catkin_LIBRARIES})
add_dependencies(lpv_lqi_qp_controller ${catkin_EXPORTED_TARGETS})

add_library(lqi_null_controller src/control/lqi_null_controller.cpp)
target_link_libraries(lqi_null_controller ${catkin_LIBRARIES})
add_dependencies(lqi_null_controller ${catkin_EXPORTED_TARGETS})

add_library(hugmy_quad_bi_controller src/control/quad_bi_controller.cpp)
target_link_libraries(hugmy_quad_bi_controller ${catkin_LIBRARIES})
add_dependencies(hugmy_quad_bi_controller ${catkin_EXPORTED_TARGETS})

add_executable(random_haptics_threshold_node src/control/test/haptics_thre_test.cpp)
target_link_libraries(random_haptics_threshold_node ${catkin_LIBRARIES})

add_executable(random_haptics_feeling_node src/control/test/haptics_feeling_test.cpp)
target_link_libraries(random_haptics_feeling_node ${catkin_LIBRARIES})

add_executable(haptics_integrated_controller src/control/integrated_all.cpp
  src/control/haptics_controller.cpp
  src/control/rviz_visualizer.cpp
  )
target_link_libraries(haptics_integrated_controller ${catkin_LIBRARIES})
add_dependencies(haptics_integrated_controller ${catkin_EXPORTED_TARGETS})

add_executable(guidance_manual_controller src/control/test/haptics_guidance_manual_test.cpp
  src/control/air_pressure_controller.cpp
  src/control/haptics_controller.cpp
  src/control/rviz_visualizer.cpp
  )
target_link_libraries(guidance_manual_controller ${catkin_LIBRARIES})

add_executable(thrust_and_pressure_cal src/control/thrust_and_pressure_cal.cpp)
target_link_libraries(thrust_and_pressure_cal ${catkin_LIBRARIES})
target_include_directories(thrust_and_pressure_cal PRIVATE ${EIGEN3_INCLUDE_DIRS})


add_library(hugmy_air_pressure src/control/air_pressure_controller.cpp)
target_link_libraries(hugmy_air_pressure ${catkin_LIBRARIES})
add_dependencies(hugmy_air_pressure ${catkin_EXPORTED_TARGETS})

add_executable(air_pressure_control_node src/nodes/organize_air_pressure.cpp)
target_link_libraries(air_pressure_control_node  hugmy_air_pressure ${catkin_LIBRARIES})
add_dependencies(air_pressure_control_node ${catkin_EXPORTED_TARGETS})

add_executable(deformation_planning_node src/nodes/deformation_planning_node.cpp)
target_link_libraries(deformation_planning_node  hugmy_deformation_planning ${catkin_LIBRARIES})
add_dependencies(deformation_planning_node ${catkin_EXPORTED_TARGETS})


add_executable(joint_model src/control/joint_model.cpp)
target_link_libraries(joint_model ${catkin_LIBRARIES})
target_include_directories(joint_model PRIVATE ${EIGEN3_INCLUDE_DIRS})

add_executable(perching_node src/control/perching.cpp)
target_link_libraries(perching_node ${catkin_LIBRARIES})
add_dependencies(perching_node ${catkin_EXPORTED_TARGETS})

if(HUGMY_MUJOCO_AVAILABLE)
  # Generate the articulated MuJoCo model used by the pneumatic hardware
  # plugin.  Real-machine builds deliberately skip these generated assets.
  set(HUGMY_MUJOCO_MODEL ${PROJECT_SOURCE_DIR}/mujoco/hugmy2/robot.xml)
  add_custom_command(
    OUTPUT ${HUGMY_MUJOCO_MODEL}
    COMMAND rosrun mujoco_ros_utils mujoco_model_generator.py
            ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml
    COMMAND ${PYTHON_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/script/finalize_mujoco_model.py
            ${HUGMY_MUJOCO_MODEL}
    DEPENDS
      ${PROJECT_SOURCE_DIR}/config/mujoco_model.yaml
      ${PROJECT_SOURCE_DIR}/script/finalize_mujoco_model.py
      ${PROJECT_SOURCE_DIR}/urdf/hugmy2.mujoco.xacro
      ${PROJECT_SOURCE_DIR}/urdf/hugmy2.urdf.xacro
      ${PROJECT_SOURCE_DIR}/urdf/mesh/4S_1000mAh_battery.stl
      ${PROJECT_SOURCE_DIR}/urdf/mesh/body_with_face_mujoco.stl
      ${PROJECT_SOURCE_DIR}/urdf/mesh/cable_carrier_2.stl
      ${PROJECT_SOURCE_DIR}/urdf/mesh/cable_carrier_arm_end.stl
      ${PROJECT_SOURCE_DIR}/urdf/mesh/long_cabble_carrier_and_moter_mount.stl
      ${PROJECT_SOURCE_DIR}/urdf/mesh/prop_5inch.dae
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
  add_custom_target(generate_hugmy_mujoco_model ALL
                    DEPENDS ${HUGMY_MUJOCO_MODEL})

  # Conservative locomotion fixture: retain a human-sized elliptic section
  # but remove the forearm taper, so normal force cannot create free progress.
  set(HUGMY_HIGH_FRICTION_MUJOCO_MODEL
      ${PROJECT_SOURCE_DIR}/mujoco/hugmy2/robot_high_friction.xml)
  add_custom_command(
    OUTPUT ${HUGMY_HIGH_FRICTION_MUJOCO_MODEL}
    COMMAND ${CMAKE_COMMAND} -E copy
            ${HUGMY_MUJOCO_MODEL}
            ${HUGMY_HIGH_FRICTION_MUJOCO_MODEL}
    COMMAND ${PYTHON_EXECUTABLE}
            ${PROJECT_SOURCE_DIR}/script/finalize_mujoco_model.py
            ${HUGMY_HIGH_FRICTION_MUJOCO_MODEL}
            --forearm-profile uniform
    DEPENDS
      ${HUGMY_MUJOCO_MODEL}
      ${PROJECT_SOURCE_DIR}/script/finalize_mujoco_model.py
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})
  add_custom_target(generate_hugmy_high_friction_mujoco_model ALL
                    DEPENDS ${HUGMY_HIGH_FRICTION_MUJOCO_MODEL})
endif()

install(DIRECTORY config doc launch plugins mujoco
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(TARGETS ${HUGMY_EXPORTED_LIBRARIES}
  DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_library(hugmy_independent_arm_pressure_controller src/control/independent_arm_pressure_controller.cpp)
add_dependencies(hugmy_independent_arm_pressure_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(hugmy_independent_arm_pressure_controller ${catkin_LIBRARIES})

add_executable(independent_arm_pressure_controller_node src/nodes/independent_arm_pressure_controller_node.cpp)
add_dependencies(independent_arm_pressure_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(independent_arm_pressure_controller_node
  hugmy_independent_arm_pressure_controller ${catkin_LIBRARIES})

add_executable(joy_arm_pressure_command_node src/nodes/joy_arm_pressure_command_node.cpp)
add_dependencies(joy_arm_pressure_command_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(joy_arm_pressure_command_node ${catkin_LIBRARIES})

add_executable(joy_human_arm_locomotion_node
  src/nodes/joy_human_arm_locomotion_node.cpp)
add_dependencies(joy_human_arm_locomotion_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(joy_human_arm_locomotion_node ${catkin_LIBRARIES})

install(TARGETS joy_arm_pressure_command_node joy_human_arm_locomotion_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

catkin_install_python(PROGRAMS
  script/analyze_hugmy_sac.py
  script/arm_bend_identification.py
  script/export_hugmy_stride.py
  script/finalize_mujoco_model.py
  script/hugmy_gym_env.py
  script/train_hugmy_sac.py
  script/evaluate_hugmy_sac.py
  script/optimize_hugmy_real_stride.py
  script/optimize_inchworm_gait.py
  script/optimize_rocking_gait.py
  script/replay_hugmy_stride_mujoco.py
  script/run_hugmy_real_stride.py
  script/sweep_recovery_mujoco.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(hugmy_attitude_pressure_controller
  src/control/attitude_pressure_controller.cpp
  src/control/hybrid_locomotion.cpp)
add_dependencies(hugmy_attitude_pressure_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(hugmy_attitude_pressure_controller ${catkin_LIBRARIES})
# The bounded predictive search runs in the control thread. An unoptimized
# Debug build exhausted its 12-ms budget before pressure/timing candidates.
# Keep debug symbols, but optimize the locomotion/recovery control units.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  set_source_files_properties(src/control/hybrid_locomotion.cpp
    src/control/attitude_pressure_controller.cpp
    PROPERTIES COMPILE_FLAGS "-O2")
endif()

add_executable(attitude_pressure_controller_node src/nodes/attitude_pressure_controller_node.cpp)
add_dependencies(attitude_pressure_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(attitude_pressure_controller_node
  hugmy_attitude_pressure_controller ${catkin_LIBRARIES})
