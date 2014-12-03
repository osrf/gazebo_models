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
  class Walker : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

      // set values
      double initial_joint_angle = 0.25708;
      double initial_joint_v = -0;
      double initial_link_v = 2.125;
      double initial_model_roll = .125;
      double initial_model_pos = .015;

      // set the initial roll for the model
      math::Pose pose;
      math::Vector3 rpy(initial_model_roll, 0.0, 0.0);
      math::Vector3 pos(0.0, 0.0, initial_model_pos);
      pose.pos = pos;
      pose.rot = math::Quaternion::EulerToQuaternion(rpy);
      this->model->SetWorldPose(pose);

      // get the front hip joint and set the position
      JointPtr joint = this->model->GetJoint("front-hip-joint");
      joint->SetPosition(0, -initial_joint_angle);
      joint->SetVelocity(0, initial_joint_v);

      // get the back legs link and set the forward velocity
      LinkPtr link = this->model->GetLink("codpiece");
      link->SetLinearVel(math::Vector3(0, -initial_link_v, 0));
    }

    // Pointer to the model
    private: physics::ModelPtr model;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Walker)
}

