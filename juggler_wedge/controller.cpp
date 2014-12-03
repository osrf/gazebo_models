#include <fstream>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <boost/foreach.hpp>
#include <boost/lexical_cast.hpp>

using gazebo::physics::JointPtr;
using gazebo::physics::LinkPtr;

namespace gazebo
{
  class Juggle : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&Juggle::OnUpdate, this, _1));

      // reset the state of the simulation
      model->GetWorld()->Reset(); 

      // set parameter values
      toff = 0.0;
      limit = -.086394;
      period = 8.0;

      // get the joint and set the position
      JointPtr joint = this->model->GetJoint("juggler-joint");
      const double LL = -1.5708 + limit;
      const double UL = 0 - limit;
      const double HALFIVAL = (UL - LL)/2.0;; 
      double q_des = std::sin(period*(toff))*HALFIVAL - HALFIVAL - limit; 
      double qd_des = std::cos(period*(toff))*HALFIVAL;
      joint->SetAngle(0, math::Angle(q_des));
      joint->SetVelocity(0, qd_des);
    }

     // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      const double KP = 1000, KV = 200;

      // get the current time
      double t = this->model->GetWorld()->GetSimTime().Double();

      // get the joint
      JointPtr joint = this->model->GetJoint("juggler-joint");

      // setup the desired position and velocity
      const double LL = -1.5708 + limit;
      const double UL = 0 - limit;
      const double HALFIVAL = (UL - LL)/2.0;; 
      double q_des = std::sin(period*(t+toff))*HALFIVAL - HALFIVAL - limit; 
      double qd_des = std::cos(period*(t+toff))*HALFIVAL;

      // compute the position and velocity error
      double perr = (q_des - joint->GetAngle(0).Radian());
      double derr = (qd_des - joint->GetVelocity(0));

      // set the control torque
      joint->SetForce(0, perr*KP + derr*KV);
    }

    // the limit for the model
    double limit;

    // the period for the model
    double period; 

    // the time offset for the controller
    double toff;
  
    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Juggle)
}

