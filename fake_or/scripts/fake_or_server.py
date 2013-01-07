#!/usr/bin/env python
import roslib; roslib.load_manifest('fake_or')
import rospy
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String
import actionlib
from object_recognition_msgs.msg import *
import sys

"""
object position: x y z

1: 0.344  0.237 0.3
2: 0.465  0.24  0.3
3: 0.553 -0.099 0.3
4: 0.424 -0.13  0.3

"""

class RecognitionServer:
    def __init__(self):
       
        #the results for the object recognition pipeline
        self.result = ObjectRecognitionResult()
        
        # on pos 1
        robj = RecognizedObject()
        robj.id.id = "IKEA-CUP-SOLBRAEND-BLUE"
        robj.pose.header.stamp = rospy.Time.now()
        robj.pose.header.frame_id = "base_link"
        robj.pose.pose.pose.position.x = 0.344
        robj.pose.pose.pose.position.y = 0.237
        robj.pose.pose.pose.position.z = 0.3
        robj.pose.pose.pose.orientation.x = 0
        robj.pose.pose.pose.orientation.y = 0
        robj.pose.pose.pose.orientation.z = 0
        robj.pose.pose.pose.orientation.w = 1
        robj.confidence = 1.0 # 50/50
        self.result.recognized_objects.objects.append(robj)
        
        # on pos 3
        robj = RecognizedObject()
        robj.id.id = "COCA-COLA-CAN-250ML"
        robj.pose.header.stamp = rospy.Time.now()
        robj.pose.header.frame_id = "base_link"
        robj.pose.pose.pose.position.x = 0.553
        robj.pose.pose.pose.position.y = -0.099
        robj.pose.pose.pose.position.z = 0.3
        robj.pose.pose.pose.orientation.x = 0
        robj.pose.pose.pose.orientation.y = 0
        robj.pose.pose.pose.orientation.z = 0
        robj.pose.pose.pose.orientation.w = 1
        robj.confidence = 1.0 # 50/50
        self.result.recognized_objects.objects.append(robj) 
        
        #actionlib stuff
        self.server = actionlib.SimpleActionServer('object_recognition', ObjectRecognitionAction, self.execute, False)
        self.server.start()
        rospy.loginfo("fake object recognition server started")


    def execute(self, goal):
        rospy.loginfo("got request")
        #TODO: add a proper header?? result.header.stamp = rospy.get_rostime()
        
        #we have a result!
        self.server.set_succeeded(result=self.result)

if __name__ == '__main__':
    rospy.init_node('fake_or_node')
    server = RecognitionServer()
    rospy.spin()