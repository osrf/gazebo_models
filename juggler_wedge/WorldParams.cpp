#include <boost/foreach.hpp>
#include <iostream>
#include <sstream>
#include <sdf/sdf.hh>
#include "gazebo/gazebo.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
//#include "gazebo/physics/bullet/BulletSurfaceParams.hh"
#include "gazebo/physics/ode/ODESurfaceParams.hh"
#include "Param.h"
#include "Counter.h"

using namespace gazebo;

// tokenizes a string
std::list<std::string> tokenize(const std::string& s)
{
  std::list<std::string> l;

  // tokenize the string
  std::istringstream input(s.c_str());
  while (true)
  {
    // get the next
    std::string next;
    input >> next;
    if (!input)
      break;
    l.push_back(next);
  }

  // transform each string to lower case
  BOOST_FOREACH(std::string& s, l)
    std::transform(s.begin(), s.end(), s.begin(), ::tolower);

  return l;
}

// sets parameters for the physics engine
void set_world_params(physics::WorldPtr world, const std::list<gazebo::Param>& params, const Counter& counter)
{
  const unsigned NPARAMS = params.size();
  unsigned int_value;
  double real_value;

  // get the physics engine
  gazebo::physics::PhysicsEnginePtr physics_engine = world->GetPhysicsEngine();

  // loop through all parameters
  std::list<gazebo::Param>::const_iterator param_iter = params.begin();
  for (unsigned i=0; i< NPARAMS; i++, param_iter++)
  {      
    // get the i'th parameter
    const gazebo::Param& p = *param_iter;

    // get the counter value for this index
    unsigned value = counter.get()[i];

    // get the parameter type
    if (p.ptype == gazebo::Param::eInt)
      int_value = p.int_start + value;
    else
      real_value = p.r_start + p.r_step*value;

    // look for integer solver parameters
    if (p.name == "iters" || p.name == "precon_iters" ||
        p.name == "max_contacts")
      physics_engine->SetParam(p.name, boost::any(int_value)); 
     
    // handle the parameter value appropriately - we could conceivably use
    // PhysicsEngine::SetParam(.) for this, but we need to know how many
    // parameters we have
    if (p.name == "cfm" || p.name == "erp" || 
        p.name == "sor" || p.name == "contact_max_correcting_vel" ||
        p.name == "min_step_size" || p.name == "max_step_size" ||
        p.name == "contact_surface_layer")
      physics_engine->SetParam(p.name, boost::any(real_value));

    // handle surface parameters
    if (p.name == "mu" || p.name == "bounce")
    {
      std::string collision_id = p.options.front();
      physics::EntityPtr entity = world->GetEntity(collision_id);
      physics::CollisionPtr collision = boost::dynamic_pointer_cast<physics::Collision>(entity);
      physics::SurfaceParamsPtr surface = collision->GetSurface();

      // check for ODE surface parameters
      boost::shared_ptr<physics::ODESurfaceParams> ode_surface = boost::dynamic_pointer_cast<physics::ODESurfaceParams>(surface);
      if (ode_surface)
      {
        if (p.name == "mu")
        {
          ode_surface->frictionPyramid.SetMuPrimary(real_value);
          ode_surface->frictionPyramid.SetMuSecondary(real_value);
        }
        else if (p.name == "bounce")
          ode_surface->bounce = real_value;
      }

/*
      // check for Bullet surface parameters 
      physics::BulletSurfaceParamsPtr bullet_surface = boost::dynamic_pointer_cast<physics::BulletSurfaceParams>(surface);
      if (bullet_surface)
      {
      }
*/
    }
  }
}

/// Reads parameters from a file and returns the list of parameters 
std::list<gazebo::Param> read_params(const char* fname)
{
  std::list<gazebo::Param> params;

  // open the file for reading
  std::ifstream in(fname);

  // continue reading until we can read no more
  while (true)
  {
    // read in a line of input
    std::string input;
    std::getline(in, input);
    if (!in)
      break;

    // tokenize the input
    std::list<std::string> tokens = tokenize(input);
    if (tokens.empty())
      continue;

    // create the parameter
    params.push_back(gazebo::Param::build_param(tokens));
  }

  // close the file
  in.close();

  return params;
}


