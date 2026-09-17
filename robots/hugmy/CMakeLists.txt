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
  mujoco_ros_control
  visualization_msgs
  )

find_package(PythonInterp REQUIRED)

catkin_package(
  LIBRARIES hugmy_pneumatic_mujoco_hw_sim
  CATKIN_DEPENDS aerial_robot_control pluginlib aerial_robot_model aerial_robot_simulation mujoco_ros_control spinal std_msgs std_srvs geometry_msgs tf visualization_msgs
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(hugmy_pneumatic_mujoco_hw_sim src/mujoco/hugmy_pneumatic_hw_sim.cpp)
add_dependencies(hugmy_pneumatic_mujoco_hw_sim ${catkin_EXPORTED_TARGETS})
target_compile_definitions(hugmy_pneumatic_mujoco_hw_sim PRIVATE SIMULATION)
target_link_libraries(hugmy_pneumatic_mujoco_hw_sim ${catkin_LIBRARIES})

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

# Generate the articulated MuJoCo model used by the pneumatic hardware plugin.
set(HUGMY_MUJOCO_MODEL ${PROJECT_SOURCE_DIR}/mujoco/hugmy2/robot.xml)
add_custom_command(
  OUTPUT ${HUGMY_MUJOCO_MODEL}
  COMMAND rosrun mujoco_ros_control mujoco_model_generator.py
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
add_custom_target(generate_hugmy_mujoco_model ALL DEPENDS ${HUGMY_MUJOCO_MODEL})

install(DIRECTORY config launch plugins mujoco
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
  )

install(TARGETS hugmy_pneumatic_mujoco_hw_sim
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

catkin_install_python(PROGRAMS
  script/analyze_hugmy_sac.py
  script/arm_bend_identification.py
  script/finalize_mujoco_model.py
  script/hugmy_gym_env.py
  script/train_hugmy_sac.py
  script/evaluate_hugmy_sac.py
  script/optimize_inchworm_gait.py
  script/optimize_rocking_gait.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_library(hugmy_attitude_pressure_controller src/control/attitude_pressure_controller.cpp)
add_dependencies(hugmy_attitude_pressure_controller ${catkin_EXPORTED_TARGETS})
target_link_libraries(hugmy_attitude_pressure_controller ${catkin_LIBRARIES})

add_executable(attitude_pressure_controller_node src/nodes/attitude_pressure_controller_node.cpp)
add_dependencies(attitude_pressure_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(attitude_pressure_controller_node
  hugmy_attitude_pressure_controller ${catkin_LIBRARIES})
