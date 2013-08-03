#!/usr/bin/env python
import roslib; roslib.load_manifest('gazebo_object_tracker')
import rospy
from geometry_msgs.msg import Pose
from gazebo.srv import GetWorldProperties, GetModelState
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
        
        self.objs = None
        self.objDict = {'COCA-COLA-CAN-250ML' : 1, 'IKEA-CUP-SOLBRAEND-BLUE' : 2}
        
        rospy.wait_for_service('/gazebo/get_world_properties')
        rospy.wait_for_service('/gazebo/get_model_state')
        
        self.gazebo_world_prop = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties, persistent=True)
        self.gazebo_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState, persistent=True)
        
        rospy.loginfo("gazebo object tracker started")
        
    def load_id_and_pose_from_gazebo(self):
      # init our members
      self.objs = {}
      # find objects in the world
      world = self.gazebo_world_prop()

      for obj in world.model_names:
        model_state = self.gazebo_model_state(model_name=obj)
        self.objs[str(obj)] = model_state.pose

    def publish(self):
        pub = rospy.Publisher('ar_pose_marker', ARMarkers)
        seq = 0
        while not rospy.is_shutdown():
            self.load_id_and_pose_from_gazebo()
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
            
            rospy.sleep(1 / PUB_FREQ)
            
if __name__ == '__main__':
    tracker = GazeboObjectTracker()
    try:
        tracker.publish()
    except rospy.ROSInterruptException:
        pass
