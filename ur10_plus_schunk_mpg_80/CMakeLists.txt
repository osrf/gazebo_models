cmake_minimum_required( VERSION 2.8 )

project( grasp_and_move )

# look for Gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

set( LIBS 
  ${GAZEBO_LIBRARIES} 
) 

add_library( grasp_and_move SHARED grasp_and_move.cpp )
target_link_libraries( grasp_and_move ${LIBS} )


