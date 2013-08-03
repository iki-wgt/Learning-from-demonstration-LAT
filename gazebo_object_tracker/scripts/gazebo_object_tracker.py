#!/usr/bin/env python
import roslib; roslib.load_manifest('gazebo_object_tracker')
import rospy
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates, ModelState
from ar_pose.msg import ARMarkers, ARMarker
from ar_track_alvar.msg import AlvarMarkers, AlvarMarker

FRAME_ID = "/base_link"
PUB_FREQ = 25.0

"""
COCA-COLA-CAN-250ML Marker 1
IKEA-CUP-SOLBRAEND-BLU Marker 2

"""

class GazeboObjectTracker:
    def __init__(self):
        rospy.init_node('gazebo_object_tracker')
        
        self.objs = {}
        self.objDict = {'COCA-COLA-CAN-250ML' : 1, 'IKEA-CUP-SOLBRAEND-BLUE' : 2}
        
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        
        rospy.loginfo("gazebo object tracker started")

    def publish(self):
        pub = rospy.Publisher('ar_pose_marker', ARMarkers)
        seq = 0
        while not rospy.is_shutdown():
            markers = ARMarkers()
            markers.header.stamp.secs = rospy.get_time()
            markers.header.seq = seq
            markers.header.frame_id = FRAME_ID
            
            for name in self.objs.keys():
                if name in self.objDict.keys():
                    marker = ARMarker()
                    marker.id = self.objDict[name]
                    marker.pose.pose = self.objs[name]
                    
                    marker.confidence = 1
                    marker.header.frame_id = FRAME_ID
                    
                    markers.markers.append(marker)
            
            pub.publish(markers)
            seq = seq + 1
            
            # no spinning needed in rospy
            rospy.sleep(1 / PUB_FREQ)
     
    def callback(self, data):
         for objIdx in range(len(data.name)):
             self.objs[str(data.name[objIdx])] = data.pose[objIdx]
     
if __name__ == '__main__':
    tracker = GazeboObjectTracker()
    try:
        tracker.publish()
    except rospy.ROSInterruptException:
        pass
