#include <iostream>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include "gazebo/physics/bullet/BulletLink.hh"
#include "gazebo/physics/ode/ODESurfaceParams.hh"
#include "gazebo/physics/bullet/BulletSurfaceParams.hh"
#include <boost/foreach.hpp>

using gazebo::physics::JointPtr;

namespace gazebo
{
  class SchaalJuggler : public ModelPlugin
  {
    private: void set_mu_and_bounce(double mu, double bounce)
    {
      physics::WorldPtr world = model->GetWorld();
      std::string collision_id = "ball-geom"; 
      physics::EntityPtr entity = world->GetEntity(collision_id);
      physics::CollisionPtr collision = boost::dynamic_pointer_cast<physics::Collision>(entity);
      physics::SurfaceParamsPtr surface = collision->GetSurface();

      // check for ODE surface parameters
      boost::shared_ptr<physics::ODESurfaceParams> ode_surface = boost::dynamic_pointer_cast<physics::ODESurfaceParams>(surface);
      if (ode_surface)
      {
        ode_surface->frictionPyramid.SetMuPrimary(mu);
        ode_surface->frictionPyramid.SetMuSecondary(mu);
        ode_surface->bounce = bounce;
      }

      // check for Bullet surface parameters 
      boost::shared_ptr<physics::BulletSurfaceParams> bullet_surface = boost::dynamic_pointer_cast<physics::BulletSurfaceParams>(surface);
      if (bullet_surface)
      {
        bullet_surface->frictionPyramid.SetMuPrimary(mu);
        bullet_surface->frictionPyramid.SetMuSecondary(mu);

        // NOTE: as of Gazebo 4.0, there is no Gazebo interface to Bullet's
        // restitution model; the following code is necessary
        physics::LinkPtr link = collision->GetLink();
        boost::shared_ptr<physics::BulletLink> blink = boost::dynamic_pointer_cast<physics::BulletLink>(link);
        btRigidBody* btrb = blink->GetBulletLink();
        btCollisionObject* bto = dynamic_cast<btCollisionObject*>(btrb); 
        bto->setRestitution(bounce);
      }
    }

    // sets the parameters depending on the counter
    private: void set_params()
    {
      const double MU = 0.0;
      const double BOUNCE = 1.0;
      const double LIMIT = 0.59;
      const double PERIOD = 2.0;

      // set mu and bounce to constants 
      set_mu_and_bounce(MU, BOUNCE);

      // set the limit and period
      limit = LIMIT;
      period = PERIOD;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&SchaalJuggler::OnUpdate, this, _1));

      // set the physics and model parameters
      set_params();

      // get the joint and set the position and velocity
      JointPtr joint = this->model->GetJoint("juggler-joint");
      double q_des = std::sin(0)*limit;
      double qd_des = std::cos(0)*limit*period;
      joint->SetAngle(0, math::Angle(q_des));
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
      const double KP = 100, KV = 20;

      // get the current time
      double t = this->model->GetWorld()->GetSimTime().Double();

      // get the joint
      JointPtr joint = this->model->GetJoint("juggler-joint");

      // setup the desired position and velocity
      double q_des = std::sin(period*t)*limit;
      double qd_des = std::cos(period*t)*limit*period;

      // compute the position and velocity error
      double perr = (q_des - joint->GetAngle(0).Radian());
      double derr = (qd_des - joint->GetVelocity(0));

      // set the control torque
      joint->SetForce(0, perr*KP + derr*KV);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // the period for the juggler
    private: double period;  

    // the upper joint limit for the juggler
    private: double limit;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(SchaalJuggler)
}

