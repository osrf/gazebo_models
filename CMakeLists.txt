cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
endif(COMMAND CMAKE_POLICY)

set (CMAKE_INSTALL_PREFIX /usr CACHE STRING "Install path prefix")

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
  pioneer2dx
  powerplant
  pr2
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

foreach (dir ${dirs})
  add_custom_target(${dir}
    ALL
    COMMAND tar czvf ${dir}.tar.gz ../${dir}
    COMMAND mv ${dir}.tar.gz ../${dir}/model.tar.gz)
  install (DIRECTORY ${dir} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/gazebo_models)
endforeach ()

message (STATUS "Install path: ${CMAKE_INSTALL_PREFIX}/share/gazebo_models")

# This must always be last!
include(CPack)
