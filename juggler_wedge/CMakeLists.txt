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

add_library(juggler-search SHARED controller-search.cpp WorldParams.cpp)
add_library(juggler SHARED controller.cpp WorldParams.cpp)
add_executable(split Iterate.cpp)
target_link_libraries(juggler-search ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(juggler ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

