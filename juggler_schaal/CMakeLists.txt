cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

################################
# Find bullet
# First and preferred option is to look for bullet standard pkgconfig, so
# check it first. If it is not present, check for the OSRF custom bullet2.82.pc
# file.
pkg_check_modules(BULLET bullet>=2.82)
if (NOT BULLET_BULLET)
  pkg_check_modules(BULLET bullet2.82>=2.82)
endif()

 if (BULLET_FOUND)
  set (HAVE_BULLET TRUE)
   add_definitions (-DHAVE_BULLET)
   add_definitions( -DLIBBULLET_VERSION=${BULLET_VERSION} )
   include_directories(${BULLET_INCLUDE_DIRS})
   link_directories(${BULLET_LIBRARY_DIRS})
   add_definitions(${BULLET_CFLAGS})
else()
  set (HAVE_BULLET FALSE)
  add_definitions( -DLIBBULLET_VERSION=0.0 )
  BUILD_WARNING ("Bullet > 2.82 not found, for bullet physics engine option, please install libbullet2.82-dev.")
endif()

add_library(juggler SHARED controller.cpp WorldParams.cpp)
target_link_libraries(juggler ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

