#!/usr/bin/env python
import roslib; roslib.load_manifest("gazebo_ros")
import sys
import os
import time
import rospy
from gazebo_msgs.msg import *
from gazebo_msgs.srv import *

def clear_joint_forces_client(joint_name):
    print "Clear ",joint_name, " effort"
    rospy.wait_for_service('/gazebo/clear_joint_forces')
    try:
      set_joint_effort = rospy.ServiceProxy('/gazebo/clear_joint_forces',JointRequest)
      resp = set_joint_effort(joint_name)
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

def set_joint_effort_client(joint_name, effort, start_time, duration):
    print "Setting ",joint_name, " to ", effort
    rospy.wait_for_service('/gazebo/apply_joint_effort')
    try:
      set_joint_effort = rospy.ServiceProxy('/gazebo/apply_joint_effort',ApplyJointEffort)
      resp = set_joint_effort(joint_name, effort, start_time, duration)
      print "set state status: ", resp.status_message
      return resp.success
    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

if __name__ == '__main__':
    rospy.init_node('gazebo_test_joint')

    if len(sys.argv) == 2:
      if sys.argv[1] == "clear":
        clear_joint_forces_client("rear_left_main_wheel_joint")
        clear_joint_forces_client("rear_right_main_wheel_joint")
        clear_joint_forces_client("front_left_main_wheel_joint")
        clear_joint_forces_client("front_right_main_wheel_joint")
    elif len(sys.argv) == 5:
      joint_name = "rear_right_main_wheel_joint"
      effort     =  float(sys.argv[1])
      start_time = rospy.Duration.from_sec(0)
      duration   = rospy.Duration.from_sec(10)
      set_joint_effort_client(joint_name, effort, start_time, duration)

      joint_name = "front_right_main_wheel_joint"
      effort     =  float(sys.argv[2])
      start_time = rospy.Duration.from_sec(0)
      duration   = rospy.Duration.from_sec(10)
      set_joint_effort_client(joint_name, effort, start_time, duration)

      joint_name = "rear_left_main_wheel_joint"
      effort     =  float(sys.argv[3])
      start_time = rospy.Duration.from_sec(0)
      duration   = rospy.Duration.from_sec(10)
      set_joint_effort_client(joint_name, effort, start_time, duration)

      joint_name = "front_left_main_wheel_joint"
      effort     =  float(sys.argv[4])
      start_time = rospy.Duration.from_sec(0)
      duration   = rospy.Duration.from_sec(10)
      set_joint_effort_client(joint_name, effort, start_time, duration)
    else:
      print "for example run:"
      print "  move_wheels.py clear  # to stop applying force"
      print "  move_wheels.py 500 -500 -500 500  #to move side ways"
      print "  move_wheels.py 500 500 -500 -500  #to rotate in place"
