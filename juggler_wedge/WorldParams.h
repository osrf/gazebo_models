#ifndef _WORLD_PARAMS_H
#define _WORLD_PARAMS_H

#include <list>
#include "Param.h"
#include "Counter.h"
#include <gazebo/physics/physics.hh>

// sets parameters for the physics engine
void set_world_params(gazebo::physics::WorldPtr world, const std::list<gazebo::Param>& params, const Counter& counter);

/// Reads parameters from a file and returns the list of parameters 
std::list<gazebo::Param> read_params(const char* fname);

#endif
