/*****************************
 * Pioneer 3AT Gazebo Plugin for ROS
 * 
 * Author: Dereck Wonnacott
 *
 *****************************/

#include <stdio.h>
#include <math.h>

#include <boost/bind.hpp>
#include <gazebo.hh>
#include <physics/physics.hh>
#include <common/common.hh>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

namespace gazebo
{   
  class ROSModelPlugin : public ModelPlugin
  {
    public:
      float maxForce;    // (Nm ?)
      float wheelRadius; // (Meters)
      float wheelBase;   // Distance between opposite wheels (Meters)
      
    private: 
      // Pointer to the model
      physics::ModelPtr model;
      
      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;

      // ROS Nodehandle
      ros::NodeHandle* node;

      // ROS Subscriber (cmd_vel)
      ros::Subscriber sub;
      
      // Odometry Publishing
      tf::TransformBroadcaster* odom_broadcaster;
      ros::Timer PubTimer;
      ros::Publisher odom_pub;
      
      ros::Time current_time;
      nav_msgs::Odometry odom;
      double pos_right_prev, pos_left_prev;
  
    public: 
      ROSModelPlugin()
      {
        // Start up ROS
        std::string name = "gazebo_p3at";
        int argc = 0;
        ros::init(argc, NULL, name);
        
        maxForce    = 5.0;   // (Nm ?)
        wheelRadius = 0.110; // (Meters)
        wheelBase   = 0.250; // Distance between opposite {R/L} wheels (Meters)
      }
      
      ~ROSModelPlugin()
      {
        delete this->node;
        delete this->odom_broadcaster;
      }

      void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
      {
        // Store the pointer to the model
        this->model = _parent;
        
        // ROS Nodehandle
        this->node = new ros::NodeHandle("~");

        // Odom vars
        this->odom_broadcaster = new tf::TransformBroadcaster;
        odom_pub = this->node->advertise<nav_msgs::Odometry>("odom", 100);
        current_time = ros::Time::now();
        
        pos_right_prev = 0; 
        pos_left_prev  = 0;
        odom.pose.pose.position.x  = 0.0;
        odom.pose.pose.position.y  = 0.0;
        odom.pose.pose.position.z  = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        
        // Publish odom Timer
        this->PubTimer = this->node->createTimer(ros::Duration(0.1), &ROSModelPlugin::odomPubCB, this );

        // ROS cmd_vel Subscriber
        this->sub = this->node->subscribe<geometry_msgs::Twist>("cmd_vel", 10, &ROSModelPlugin::cmd_velCB, this );

        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&ROSModelPlugin::OnUpdate, this));
      }

      void cmd_velCB(const boost::shared_ptr<geometry_msgs::Twist const>& cmd_vel)
      {                                   
        physics::Joint_V joints = this->model->GetJoints();
        
        joints[0]->SetMaxForce(0, maxForce);
        joints[1]->SetMaxForce(0, maxForce);
        joints[2]->SetMaxForce(0, maxForce);
        joints[3]->SetMaxForce(0, maxForce);
        
        float RotVel_lin = cmd_vel->linear.x / wheelRadius;
        float RotVel_rot = cmd_vel->angular.z * (wheelBase / wheelRadius);
        
        joints[0]->SetVelocity(0, RotVel_lin - RotVel_rot); // Front Right
        joints[1]->SetVelocity(0, RotVel_lin - RotVel_rot); // Rear  Right
        joints[2]->SetVelocity(0, RotVel_lin + RotVel_rot); // Front Left
        joints[3]->SetVelocity(0, RotVel_lin + RotVel_rot); // Rear  Left
        
        //ROS_INFO("cmd_vel: [%f, %f]", cmd_vel->linear.x , cmd_vel->angular.z );
      }
      
      // Publish ROS Topic only at a specified rate
      void odomPubCB(const ros::TimerEvent& event)
      {
        odom_pub.publish(odom);
        
        // Publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;

        odom_trans.header.stamp            = current_time;
        odom_trans.header.frame_id         = "odom";
        odom_trans.child_frame_id          = "base_link";

        odom_trans.transform.translation.x = odom.pose.pose.position.x;
        odom_trans.transform.translation.y = odom.pose.pose.position.y;
        odom_trans.transform.translation.z = odom.pose.pose.position.z;
        odom_trans.transform.rotation      = odom.pose.pose.orientation;
        
        odom_broadcaster->sendTransform(odom_trans);
      }
      
      // Called by the world update start event
      void OnUpdate()
      {
        ros::spinOnce();
        
        ros::Duration dt = ros::Time::now() - current_time;
        current_time = ros::Time::now();
        
        // Update Pose Estimate and Current Velocity
        physics::Joint_V joints = this->model->GetJoints();
        double th;
        double dx, dy, dth;
        
        // Average the position of the 'encoders' on each side of the robot to compensate for slips
        double pos_right = (joints[0]->GetAngle(0).Radian() + joints[1]->GetAngle(0).Radian())/2 * wheelRadius ;
        double pos_left  = (joints[2]->GetAngle(0).Radian() + joints[3]->GetAngle(0).Radian())/2 * wheelRadius ;
        double dpos_right = pos_right_prev - pos_right;
        double dpos_left  = pos_left_prev  - pos_left;
                
        tf::Pose pose;
        tf::poseMsgToTF(odom.pose.pose, pose);
        th  = tf::getYaw(pose.getRotation());
        
        dx  = -0.5 * cos(th) * (dpos_right + dpos_left);
        dy  = -0.5 * sin(th) * (dpos_right + dpos_left);
        dth = (dpos_right - dpos_left) / (2 * wheelBase);
              
        // Publish the odometry message over a ROS topic
        odom.header.stamp          = current_time;
        odom.header.frame_id       = "odom";
        odom.child_frame_id        = "base_link";

        odom.pose.pose.position.x += dx;
        odom.pose.pose.position.y += dy;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th + dth);

        odom.twist.twist.linear.x  = dx  / dt.toSec();
        odom.twist.twist.linear.y  = dy  / dt.toSec();
        odom.twist.twist.angular.z = dth / dt.toSec();
        
        // Save the previous 'encoder' values
        pos_right_prev = pos_right;
        pos_left_prev  = pos_left;
      }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ROSModelPlugin)
}
