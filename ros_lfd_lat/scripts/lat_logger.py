#!/usr/bin/env python
import sys
from os import path, makedirs

import roslib; roslib.load_manifest('ros_lfd_lat')
import rospy
import actionlib

from std_msgs.msg import String
from object_recognition_msgs.msg import *
from sensor_msgs.msg import JointState

trajectory_name = "default_trajectory_name"
demo_name = "default_demo_name"
dirname = ""
active = False
demo_count = 0

def log_trajectory(data):
  """Callback that writes the joint positions to a file.
  
  This listens to the topic /joint_states.
  Write mode is append. The file is opened and closed with every message
  
  """
  if active is True:
    with open(str(dirname) + "/" + demo_name, "a") as dataFile:
      dof = len(data.position)
      
      for i in range(dof - 1):
        dataFile.write(str(data.position[i]) + "\t")
      
      # after the last value there is no tab but a newline
      dataFile.write(str(data.position[dof - 1]) + "\n")

def log_objects(status, result):
  """Callback for the action result of the object recognition."""
  
  with open(str(dirname) + "/" + trajectory_name + ".tra", "a") \
    as trajectoryFile:
    
    # write name of demo
    trajectoryFile.write(demo_name + "\n")
  
    for obj in result.recognized_objects.objects:
      id = obj.id.id
      x = obj.pose.pose.pose.position.x
      y = obj.pose.pose.pose.position.y
      z = obj.pose.pose.pose.position.z
      
      # skip the ground plane and the katana
      if id != "plane1_model" and id != "katana":    
        trajectoryFile.write(str(id) + "(" + str(x) + "," + str(y) + "," \
         + str(z) + ")")
    
    # finish the line with a newline
    trajectoryFile.write("\n")
  
  rospy.loginfo("objects written to file " \
    + str(dirname) + "/" + trajectory_name + ".tra")

def lat_logger():
  """Stores the joint states and objects in leatra compatible files.
  
  Creates a demo file <demo_name> with the trajectory data and appends the 
  demo file and the recognized objects to the <trajectory_name>.tra file.
  If there is no directory called <trajectory_name>.tra, this function will
  create it.
  
  """
  global dirname, active, demo_count, trajectory_name, demo_name

  
  finished = False  
  rospy.init_node('lat_logger')
  
  rospy.loginfo("ROS LfD data logger\n")
  
  # init object recognition
  object_client = actionlib.SimpleActionClient('object_recognition', ObjectRecognitionAction)
  object_client.wait_for_server()
  rospy.loginfo("object recognition server ready")
  
  # init joint state listener
  active = False  # don't record data at the moment
  rospy.Subscriber("joint_states", JointState, log_trajectory)
  
  trajectory_name = raw_input('Please enter a name for the trajectory: ')
  rospy.loginfo("Selected name for trajectory: %s", trajectory_name)
  
  # check if directory exists
  dirname = "/home/benny/lat_demos/" + trajectory_name + ".tra"
  if not path.exists(dirname):
    makedirs(dirname)
    rospy.loginfo("directory %s created", dirname)
  
  while not rospy.is_shutdown() and not finished:
    demo_count = demo_count + 1
    
    demo_name = raw_input('Please enter a name for the next demo ' \
      + '(empty string for default names): ')
    
    if demo_name == '' or demo_name == 'default_demo_name':
      demo_name = "demo" + str(demo_count)
    
    rospy.loginfo("Selected name for demo: %s", demo_name)
    
    # log objects
    object_client.send_goal(ObjectRecognitionGoal(), done_cb=log_objects)
    object_client.wait_for_result() # wait indefinitely for a result
    
    rospy.loginfo("objects recognised, now start the demonstration")
    
    # log trajectory
    active = True
    
    raw_input('Press enter to finish demo. ')
    
    active = False
    
    answer = raw_input("Enter 'yes' to show another demo? ")
    
    if answer == "yes":
      finished = False
    else:
      finished = True
    
    
    
  #rospy.spin()
  


if __name__ == '__main__':
  lat_logger()


