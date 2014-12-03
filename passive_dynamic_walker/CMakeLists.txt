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

add_library(walker-search SHARED plugin-search.cpp WorldParams.cpp)
add_library(walker SHARED plugin.cpp )
target_link_libraries(walker-search ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(walker ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

