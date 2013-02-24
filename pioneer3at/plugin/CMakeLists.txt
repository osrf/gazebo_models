cmake_minimum_required(VERSION 2.8)

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo REQUIRED)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include(cmake/rospack.cmake)

get_rospack_flags(roscpp)
include_directories(${roscpp_INCLUDE_DIRS})
link_directories(${roscpp_LIBRARY_DIRS})

get_rospack_flags(tf)
include_directories(${tf_INCLUDE_DIRS})
link_directories(${tf_LIBRARY_DIRS})


set(CMAKE_INSTALL_RPATH "${tf_LIBRARY_DIRS}")
add_library(ros_model_plugin SHARED src/ros_model_plugin.cc)
set_target_properties(ros_model_plugin PROPERTIES COMPILE_FLAGS "${roscpp_CFLAGS_OTHER}")
set_target_properties(ros_model_plugin PROPERTIES LINK_FLAGS "${roscpp_LDFLAGS_OTHER}")

target_link_libraries(ros_model_plugin ${tf_LIBRARIES})

install (TARGETS ros_model_plugin DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/gazebo_plugins/)
