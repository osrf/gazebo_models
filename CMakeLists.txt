cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

if(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
endif(COMMAND CMAKE_POLICY)

set (CMAKE_INSTALL_PREFIX /usr/share CACHE STRING "Install path prefix")

project (gazebo_models)
string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

exec_program(date ARGS -u +%Y-%m-%-dT%H:%M:%S OUTPUT_VARIABLE CURRENT_DATE)

set (MODEL_URI_LIST "" CACHE INTERNAL "Model URI List" FORCE)

# Convert install prefix to an absolute path, to support the common case of
# doing this (which otherwise will fail during the install):`
#   cmake -DCMAKE_INSTALL_PREFIX=~/.gazebo ..
get_filename_component(CMAKE_INSTALL_PREFIX_ABSOLUTE ${CMAKE_INSTALL_PREFIX}
  ABSOLUTE)

file(GLOB_RECURSE MODEL_CFGS "model.config")
foreach (MODEL_CFG ${MODEL_CFGS})
  get_filename_component(dir ${MODEL_CFG} DIRECTORY) # get dir of model.config
  get_filename_component(dir ${dir} NAME) # get dirname only, not full path
  add_custom_target(${dir} ALL COMMAND mkdir -p ${PROJECT_BINARY_DIR}/${dir}
    COMMAND tar czvf ${PROJECT_BINARY_DIR}/${dir}/model.tar.gz ../${dir})

  install (DIRECTORY ${dir} DESTINATION ${CMAKE_INSTALL_PREFIX_ABSOLUTE}/models)
  install (FILES ${PROJECT_BINARY_DIR}/${dir}/model.tar.gz DESTINATION
    ${CMAKE_INSTALL_PREFIX_ABSOLUTE}/models/${dir})
  set(MODEL_URI_LIST "${MODEL_URI_LIST}<uri>file://${dir}</uri>\n")
endforeach ()

configure_file("${CMAKE_SOURCE_DIR}/manifest.xml.in"
               "${PROJECT_BINARY_DIR}/manifest.xml")

install (FILES "${PROJECT_BINARY_DIR}/manifest.xml" DESTINATION
         ${CMAKE_INSTALL_PREFIX_ABSOLUTE}/models/)

configure_file("${CMAKE_SOURCE_DIR}/database.config.in"
               "${PROJECT_BINARY_DIR}/database.config")

install (FILES "${PROJECT_BINARY_DIR}/database.config" DESTINATION
         ${CMAKE_INSTALL_PREFIX_ABSOLUTE}/models/)

message (STATUS "Install path: ${CMAKE_INSTALL_PREFIX_ABSOLUTE}/models")

# This must always be last!
include(CPack)
