cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
endif(COMMAND CMAKE_POLICY)

set (CMAKE_INSTALL_PREFIX /usr/share CACHE STRING "Install path prefix")

project (gazebo_models)
string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

exec_program(date ARGS -u +%Y-%m-%-dT%H:%M:%S OUTPUT_VARIABLE CURRENT_DATE)

set (dirs
asphalt_plane
beer
bookshelf
bowl
brick_box_3x1x3
cabinet
camera
cinder_block
coke_can
construction_barrel
construction_cone
cordless_drill
create
cube_20k
double_pendulum_with_base
drc_practice_2x4
drc_practice_2x6
drc_practice_4x4x20
drc_practice_4x4x40
drc_practice_angled_barrier_135
drc_practice_angled_barrier_45
drc_practice_ball_valve
drc_practice_ball_valve_wall
drc_practice_base_4x8
drc_practice_block_wall
drc_practice_blue_cylinder
drc_practice_door_4x8
drc_practice_handle_wheel_valve
drc_practice_handle_wheel_valve_wall
drc_practice_hand_wheel_valve
drc_practice_hand_wheel_valve_wall
drc_practice_ladder
drc_practice_orange_jersey_barrier
drc_practice_truss
drc_practice_valve_wall
drc_practice_wheel_valve
drc_practice_wheel_valve_large
drc_practice_wheel_valve_large_wall
drc_practice_wheel_valve_wall
drc_practice_white_jersey_barrier
drc_practice_yellow_parking_block
dumpster
fast_food
fire_hose_long
gas_station
grey_wall
ground_plane
hammer
hinged_door
hokuyo
house_1
house_2
house_3
husky
irobot_hand
jersey_barrier
kinect
kitchen_dining
lamp_post
mailbox
monkey_wrench
mud_box
nist_elevated_floor_120
nist_fiducial_barrel
nist_maze_wall_120
nist_maze_wall_240
nist_maze_wall_triple_holes_120
nist_simple_ramp_120
nist_stairs_120
office_building
pioneer2dx
pioneer3at
polaris_ranger_ev
powerplant
pr2
pr2_gripper
saucepan
simple_arm
simple_arm_gripper
simple_gripper
speed_limit_sign
starting_pen
stereo_camera
stop_sign
sun
table
turtlebot
utility_cart
willowgarage
youbot
)

set (MODEL_URI_LIST "" CACHE INTERNAL "Model URI List" FORCE)

foreach (dir ${dirs})
  add_custom_target(${dir} ALL COMMAND mkdir -p ${PROJECT_BINARY_DIR}/${dir}
    COMMAND tar czvf ${PROJECT_BINARY_DIR}/${dir}/model.tar.gz ../${dir})

  install (DIRECTORY ${dir} DESTINATION ${CMAKE_INSTALL_PREFIX}/models)
  install (FILES ${PROJECT_BINARY_DIR}/${dir}/model.tar.gz DESTINATION
    ${CMAKE_INSTALL_PREFIX}/models/${dir})
  set(MODEL_URI_LIST "${MODEL_URI_LIST}<uri>file://${dir}</uri>\n")
endforeach ()

configure_file("${CMAKE_SOURCE_DIR}/manifest.xml.in"
               "${PROJECT_BINARY_DIR}/manifest.xml")

install (FILES "${PROJECT_BINARY_DIR}/manifest.xml" DESTINATION
         ${CMAKE_INSTALL_PREFIX}/models/)

configure_file("${CMAKE_SOURCE_DIR}/database.config.in"
               "${PROJECT_BINARY_DIR}/database.config")

install (FILES "${PROJECT_BINARY_DIR}/database.config" DESTINATION
         ${CMAKE_INSTALL_PREFIX}/models/)

message (STATUS "Install path: ${CMAKE_INSTALL_PREFIX}/models")

# This must always be last!
include(CPack)
