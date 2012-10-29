cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
endif(COMMAND CMAKE_POLICY)

set (CMAKE_INSTALL_PREFIX /usr/share CACHE STRING "Install path prefix")

project (gazebo_models)
string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

#include (FindPkgConfig)
#if (PKG_CONFIG_FOUND)
#  pkg_check_modules(GAZEBO gazebo)
#  pkg_check_modules(OGRE OGRE)
#  pkg_check_modules(CEGUI CEGUI)
#endif()
#
#include_directories(
#  ${GAZEBO_INCLUDE_DIRS}
#  ${OGRE_INCLUDE_DIRS}
#  ${CEGUI_INCLUDE_DIRS}
#)
#
#link_directories(
#  ${GAZEBO_LIBRARY_DIRS}
#  ${OGRE_LIBRARY_DIRS}
#  ${CEGUI_LIBRARY_DIRS}
#)


set (dirs
bookshelf
bowl
cabinet
camera
coke_can
cordless_drill
create
cube_20k
ground_plane
hammer
hokuyo
kinect
monkey_wrench
nist_elevated_floor_120
nist_fiducial_barrel
nist_maze_wall_120
nist_maze_wall_240
nist_maze_wall_triple_holes_120
nist_simple_ramp_120
nist_stairs_120
pioneer2dx
powerplant
pr2
pr2_gripper
saucepan
simple_arm
simple_arm_gripper
simple_gripper
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

message (STATUS "Install path: ${CMAKE_INSTALL_PREFIX}/models")

# This must always be last!
include(CPack)

