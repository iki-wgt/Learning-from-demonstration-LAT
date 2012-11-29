#!/usr/bin/env python
import sys
from os import path, makedirs

import rosbag

def parseBag(fileName, trajectoryName, demoName):
  """Parses a .bag file and create the Leatra compatible files.
  
  Creates a demo file <demoName> with the trajectory data and appends the 
  demo file and the recognized objects to the <trajectoryName>.tra file.
  If there is no directory called <trajectoryName>.tra, this function will
  create it.
  
  Arguments:
  fileName       -- file name of the .bag file
  trajectoryName -- name of the trajectory, this name specifies the folder
                    where the files are stored
  demoName       -- name of the demonstration recorded in this bag file
  
  """
  # check if directory exists
  dirname = trajectoryName + ".tra"
  if not path.exists(dirname):
    makedirs(dirname)
    
  # see ROS example http://www.ros.org/wiki/rosbag/Code%20API
  bag = rosbag.Bag(fileName)  # open bag file in read mode
  
  # the bag file only contains a single message from the topic /lfd_demo
  bagData = bag.read_messages("/lfd_demo")
  topic, msg, t = bagData.next()
  
  # store the joint states
  with open(str(dirname) + "/" + demoName, "w") as dataFile:
    for jointState in msg.joint_states:
      dimensions = len(jointState.position)
      
      for i in range(dimensions - 1):
        dataFile.write(str(jointState.position[i]) + "\t")
      
      # after the last value there is no tab but a newline
      dataFile.write(str(jointState.position[dimensions - 1]) + "\n")
  
  # store the objects
  with open(str(dirname) + "/" + trajectoryName + ".tra", "w") as trajectoryFile:
    # write name of demo
    trajectoryFile.write(demoName + "\n")
    
    for recObject in msg.recognized_objects.objects:
      id = recObject.id.id
      x = recObject.pose.pose.pose.position.x
      y = recObject.pose.pose.pose.position.y
      z = recObject.pose.pose.pose.position.z
      
      trajectoryFile.write(str(id) + "(" + str(x) + "," + str(y) + "," + str(z) \
        + ")")
    
    # finish the line with a newline
    trajectoryFile.write("\n")
  
  # bag file isn't needed anymore
  bag.close()
if __name__ == '__main__':
  if len(sys.argv) != 4:
    print "Usage bag2leatra.py <fileName> <trajectoryName> <demoName>"
  else:
    fileName = str(sys.argv[1])
    trajectoryName = str(sys.argv[2])
    demoName = str(sys.argv[3])
    parseBag(fileName, trajectoryName, demoName)
