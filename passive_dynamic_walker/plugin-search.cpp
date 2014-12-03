#include <fstream>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>
#include "WorldParams.h"
#include "Param.h"
#include "Counter.h"

using gazebo::physics::JointPtr;
using gazebo::physics::LinkPtr;

namespace gazebo
{
  class Walker : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&Walker::OnUpdate, this, _1));

      // read the parameters out of the input file 
      char* param_fname = getenv("GZ_BRUTE_PARAM_FILE");
      if (!param_fname)
        throw std::runtime_error("GZ_BRUTE_PARAM_FILE environment variable not set");
      params = read_params(param_fname); 

      // setup the counter for the number of parameters
      std::vector<unsigned> max;
      BOOST_FOREACH(const gazebo::Param& p, params)
      {
        if (p.ptype == gazebo::Param::eInt)
          max.push_back(p.int_end - p.int_start);
        else
          max.push_back((unsigned) ((p.r_end - p.r_start)/p.r_step));
      }
      counter.reset(max);

      // set the physics and model parameters
      set_params();

      // get the front hip joint and set the position
      JointPtr joint = this->model->GetJoint("front-hip-joint");
      joint->SetPosition(0, -initial_joint_angle);
      joint->SetVelocity(0, initial_joint_v);

      // get the back legs link and set the forward velocity
      LinkPtr link = this->model->GetLink("codpiece");
      link->SetLinearVel(math::Vector3(0, -initial_link_v, 0));
    }

   // sets the parameters depending on the counter
    private: void set_params()
    {
      const unsigned NPARAMS = params.size();
      double real_value;

      // set the world parameters
      set_world_params(model->GetWorld(), params, counter);

      // loop through all parameters
      std::list<gazebo::Param>::const_iterator param_iter = params.begin();
      for (unsigned i=0; i< NPARAMS; i++, param_iter++)
      {      
        // get the i'th parameter
        const gazebo::Param& p = *param_iter;

        // get the counter value for this index
        unsigned value = counter.get()[i];

        // get the parameter type
        if (p.ptype != gazebo::Param::eReal)
          continue; 
        real_value = p.r_start + p.r_step*value;

        // look for the limit, period, and time offset
        if (p.name == "joint-angle")
          initial_joint_angle = real_value;
        if (p.name == "joint-velocity")
          initial_joint_v = real_value;
        if (p.name == "link-velocity")
          initial_link_v = real_value;
      }
    }

    /// Gets the current parameters as a string
    private: std::string get_params()
    {
      const unsigned NPARAMS = params.size();
      std::string str;
      std::string string_value;

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
          string_value = boost::lexical_cast<std::string>(p.int_start + value);
        else if (p.ptype == gazebo::Param::eReal)
          string_value = boost::lexical_cast<std::string>(p.r_start + p.r_step*value);

        if (i != 0)
          str = str + " ";
        str = str + p.name + ":" + string_value;
        BOOST_FOREACH(const std::string& s, p.options)
          str = str + ":" + s;
      }

      return str;
    }

    // determines whether the robot has fallen over (returns 'true' if not) 
    public: bool CheckGoal()
    {
      const unsigned Z = 2;

      // verify that the position on the ball is not below zero 
      LinkPtr link = this->model->GetLink("codpiece");
      math::Pose pose = link->GetWorldCoGPose();
      return (pose.pos[Z] >= 0.5 && model->GetWorld()->GetIterations() < 20000);
    }
    
     // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      // check the goal
      if (!CheckGoal())
      {
        // open the output file
        std::ofstream out;
        if (getenv("GZ_BRUTE_OUTPUT_FILE"))
          out.open(getenv("GZ_BRUTE_OUTPUT_FILE"), std::ofstream::app);
        else
          out.open("/dev/stdout");

        // write out the current parameters
        out << get_params() << " " << model->GetWorld()->GetIterations() << std::endl;
        out.close();

        // see whether the counter is full
        if (counter.full())
          exit(0);

        // set the next set of parameters
        counter++;
        set_params();

        // reset the state of the simulation
        model->GetWorld()->Reset(); 

        // get the front hip joint and set the position
        JointPtr joint = this->model->GetJoint("front-hip-joint");
        joint->SetPosition(0, -initial_joint_angle);
        joint->SetVelocity(0, initial_joint_v);

        // get the back legs link and set the forward velocity
        LinkPtr link = this->model->GetLink("codpiece");
        link->SetLinearVel(math::Vector3(0, -initial_link_v, 0));
      }
    }

    // initial joint angle
    private: double initial_joint_angle;

    // initial joint velocity
    private: double initial_joint_v;

    // initial link velocity (in the -y direction)
    private: double initial_link_v;

    // Pointer to the model
    private: physics::ModelPtr model;

    // pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // counter
    private: Counter counter;

    // list of parameters
    private: std::list<gazebo::Param> params;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Walker)
}

