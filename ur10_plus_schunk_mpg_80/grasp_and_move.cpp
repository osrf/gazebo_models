#include "grasp_and_move.h"
#include <stdexcept>

//-----------------------------------------------------------------------------

GZ_REGISTER_WORLD_PLUGIN( grasp_and_move_c )


//-----------------------------------------------------------------------------
grasp_and_move_c::grasp_and_move_c( void ) { 

}

//-----------------------------------------------------------------------------
grasp_and_move_c::~grasp_and_move_c( void ) {
    gazebo::event::Events::DisconnectWorldUpdateBegin( _updateConnection );
}

//-----------------------------------------------------------------------------
void grasp_and_move_c::Load( gazebo::physics::WorldPtr world, sdf::ElementPtr sdf ) {

  // -- PRINCIPLE REFERENCES --
  _world = world;
  _model = world->GetModel("ur10_plus_schunk_mpg_80");

  // verify model exists
  if (!_model)
    throw std::runtime_error("Expected ur10_plus_schunk_mpg_80 model!");

  // -- COMPONENT REFERENCES --
  // ---- UR10 ----
  // link references
  _base = _model->GetLink("base_link");
  if( !_base ) {
    gzerr << "Unable to find link: base_link\nPlugin failed to load\n";
    return;
  }

  _shoulder = _model->GetLink("shoulder_link");
  if( !_shoulder ) {
    gzerr << "Unable to find link: shoulder_link\nPlugin failed to load\n";
    return;
  }

  _upperarm = _model->GetLink("upper_arm_link");
  if( !_upperarm ) {
    gzerr << "Unable to find link: upper_arm_link\nPlugin failed to load\n";
    return;
  }

  _forearm = _model->GetLink("forearm_link");
  if( !_forearm ) {
    gzerr << "Unable to find link: forearm_link\nPlugin failed to load\n";
    return;
  }

  _wrist1 = _model->GetLink("wrist_1_link");
  if( !_wrist1 ) {
    gzerr << "Unable to find link: wrist_1_link\nPlugin failed to load\n";
    return;
  }

  _wrist2 = _model->GetLink("wrist_2_link");
  if( !_wrist2 ) {
    gzerr << "Unable to find link: wrist_2_link\nPlugin failed to load\n";
    return;
  }

  _wrist3 = _model->GetLink("wrist_3_link");
  if( !_wrist3 ) {
    gzerr << "Unable to find link: wrist_3_link\nPlugin failed to load\n";
    return;
  }

  // joint references
  _shoulder_pan_actuator = _model->GetJoint("shoulder_pan_joint");
  if( !_shoulder_pan_actuator ) {
    gzerr << "Unable to find joint: shoulder_pan_joint\nPlugin failed to load\n";
    return;
  } 

  _shoulder_lift_actuator = _model->GetJoint("shoulder_lift_joint");
  if( !_shoulder_lift_actuator ) {
    gzerr << "Unable to find joint: shoulder_lift_joint\nPlugin failed to load\n";
    return;
  } 

  _elbow_actuator = _model->GetJoint("elbow_joint");
  if( !_elbow_actuator ) {
    gzerr << "Unable to find joint: elbow_joint\nPlugin failed to load\n";
    return;
  } 

  _wrist1_actuator = _model->GetJoint("wrist_1_joint");
  if( !_wrist1_actuator ) {
    gzerr << "Unable to find joint: wrist_1_joint\nPlugin failed to load\n";
    return;
  } 

  _wrist2_actuator = _model->GetJoint("wrist_2_joint");
  if( !_wrist2_actuator ) {
    gzerr << "Unable to find joint: wrist_2_joint\nPlugin failed to load\n";
    return;
  } 

  _wrist3_actuator = _model->GetJoint("wrist_3_joint");
  if( !_wrist3_actuator ) {
    gzerr << "Unable to find joint: wrist_3_joint\nPlugin failed to load\n";
    return;
  } 

  // ---- SCHUNK ----
  // link references
  _hand = _model->GetLink("hand");
  if( !_hand ) {
    gzerr << "Unable to find link: hand\nPlugin failed to load\n";
    return;
  }

  _finger_l = _model->GetLink("l_finger");
  if( !_finger_l ) {
    gzerr << "Unable to find link: l_finger\nPlugin failed to load\n";
    return;
  }

  _finger_r = _model->GetLink("r_finger");
  if( !_finger_r ) {
    gzerr << "Unable to find link: r_finger\nPlugin failed to load\n";
    return;
  }

  // joint references
  _finger_actuator_l = _model->GetJoint("l_finger_actuator");
  if( !_finger_actuator_l ) {
    gzerr << "Unable to find joint: l_finger_actuator\nPlugin failed to load\n";
    return;
  } 

  _finger_actuator_r = _model->GetJoint("r_finger_actuator");
  if( !_finger_actuator_r ) {
    gzerr << "Unable to find joint: r_finger_actuator\nPlugin failed to load\n";
    return;
  } 

  // reset the world before we begin
  _model->GetWorld()->Reset();  

  // set the starting velocity for the joints
  const double PERIOD = 5.0;
  const double AMP = 0.5;
  const double SMALL_AMP = AMP*0.1;
  _shoulder_pan_actuator->SetVelocity(0, std::cos(0)*AMP*PERIOD);
  _shoulder_lift_actuator->SetVelocity(0, std::cos(0)*SMALL_AMP*PERIOD*2.0);
  _elbow_actuator->SetVelocity(0, std::cos(0)*AMP*PERIOD*2.0/3.0);
  _wrist1_actuator->SetVelocity(0, std::cos(0)*AMP*PERIOD*1.0/7.0);
  _wrist2_actuator->SetVelocity(0, std::cos(0)*AMP*PERIOD*2.0/11.0);
  _wrist3_actuator->SetVelocity(0, std::cos(0)*AMP*PERIOD*3.0/13.0);

  // -- CALLBACKS --
  _updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
    boost::bind( &grasp_and_move_c::Update, this ) );

  // -- FIN --
  printf( "gazebo/grasp_and_move has initialized\n" );
}

//-----------------------------------------------------------------------------
void grasp_and_move_c::Update( ) {

  // get the current time
  double t = _model->GetWorld()->GetSimTime().Double();

  // determinet the desired position and velocity for the controller 
  const double PERIOD = 5.0;
  const double AMP = 0.5;
  const double SMALL_AMP = AMP*0.1;
  double sh_pan_q_des = std::sin(t)*AMP*PERIOD;
  double sh_pan_qd_des = std::cos(t)*AMP*PERIOD;
  double sh_lift_q_des = std::sin(t*2.0)*SMALL_AMP*PERIOD*2.0;
  double sh_lift_qd_des = std::cos(t*2.0)*SMALL_AMP*PERIOD*2.0;
  double elbow_q_des = std::sin(t*2.0/3.0)*AMP*PERIOD*2.0/3.0;
  double elbow_qd_des = std::cos(t*2.0/3.0)*AMP*PERIOD*2.0/3.0;
  double wrist1_q_des = std::sin(t*1.0/7.0)*AMP*PERIOD*1.0/7.0;
  double wrist1_qd_des = std::cos(t*1.0/7.0)*AMP*PERIOD*1.0/7.0;
  double wrist2_q_des = std::sin(t*2.0/11.0)*AMP*PERIOD*2.0/11.0;
  double wrist2_qd_des = std::cos(t*2.0/11.0)*AMP*PERIOD*2.0/11.0;
  double wrist3_q_des = std::sin(t*3.0/13.0)*AMP*PERIOD*3.0/13.0;
  double wrist3_qd_des = std::cos(t*3.0/13.0)*AMP*PERIOD*3.0/13.0;

  // compute the errors
  double sh_pan_q_err = (sh_pan_q_des - _shoulder_pan_actuator->GetAngle(0).Radian());
  double sh_pan_qd_err = (sh_pan_qd_des - _shoulder_pan_actuator->GetVelocity(0));
  double sh_lift_q_err = (sh_lift_q_des - _shoulder_lift_actuator->GetAngle(0).Radian());
  double sh_lift_qd_err = (sh_lift_qd_des - _shoulder_lift_actuator->GetVelocity(0));
  double elbow_q_err = (elbow_q_des - _elbow_actuator->GetAngle(0).Radian());
  double elbow_qd_err = (elbow_qd_des - _elbow_actuator->GetVelocity(0));
  double wrist1_q_err = (wrist1_q_des - _wrist1_actuator->GetAngle(0).Radian());
  double wrist1_qd_err = (wrist1_qd_des - _wrist1_actuator->GetVelocity(0));
  double wrist2_q_err = (wrist2_q_des - _wrist2_actuator->GetAngle(0).Radian());
  double wrist2_qd_err = (wrist2_qd_des - _wrist2_actuator->GetVelocity(0));
  double wrist3_q_err = (wrist3_q_des - _wrist3_actuator->GetAngle(0).Radian());
  double wrist3_qd_err = (wrist3_qd_des - _wrist3_actuator->GetVelocity(0));

  // setup gains
  const double SH_KP = 300.0, SH_KV = 120.0;
  const double EL_KP = 60.0, EL_KV = 24.0;
  const double WR_KP = 15.0, WR_KV = 6.0;
 
  // compute the actuator forces
  double sh_pan_f = SH_KP*sh_pan_q_err + SH_KV*sh_pan_qd_err;
  double sh_lift_f = SH_KP*sh_lift_q_err + SH_KV*sh_lift_qd_err;
  double elbow_f = EL_KP*elbow_q_err + EL_KV*elbow_qd_err;
  double wrist1_f = WR_KP*wrist1_q_err + WR_KV*wrist1_qd_err;
  double wrist2_f = WR_KP*wrist2_q_err + WR_KV*wrist2_qd_err;
  double wrist3_f = WR_KP*wrist3_q_err + WR_KV*wrist3_qd_err;

  // set the actuator forces for the arm
  _shoulder_pan_actuator->SetForce(0, sh_pan_f);
  _shoulder_lift_actuator->SetForce(0, sh_lift_f);
  _elbow_actuator->SetForce(0, elbow_f);
  _wrist1_actuator->SetForce(0, wrist1_f);
  _wrist2_actuator->SetForce(0, wrist2_f);
  _wrist3_actuator->SetForce(0, wrist3_f);

  // Simple close fingers test case
  _finger_actuator_l->SetForce(0, 100.0);  // close
  //_finger_actuator_l->SetForce(0, -0.00001);  // open

  _finger_actuator_r->SetForce(0,-100.0); // close
  //_finger_actuator_r->SetForce(0,0.00001);  //open

}

//-----------------------------------------------------------------------------
/*
void grasp_and_move_c::Reset( ) {

}
*/
//-----------------------------------------------------------------------------
