#!/usr/bin/env python

'''
Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Hutmacher Robin, Karrenbauer Oliver, Meissner Pascal, Schleicher Ralf, Stoeckle Patrick, Trautmann Jeremias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import roslib
import rospy
import numpy 
import tf
import tf2_ros
import time

from geometry_msgs.msg import (Pose, PoseWithCovariance, PoseWithCovarianceStamped, Point, Quaternion, Twist, Vector3)
from asr_msgs.msg import AsrObject
from actionlib import *
from actionlib.msg import *
from asr_robot_model_services.msg import RobotStateMessage
from asr_robot_model_services.srv import CalculateCameraPose
from next_best_view.srv import TriggerFrustumVisualization
from sensor_msgs.msg import JointState
from asr_world_model.srv import PushFoundObject, PushFoundObjectList

detected_objects = []

def get_camera_pose_cpp():
    """
    Returns camera pose
    """
    try:
        rospy.wait_for_service('/asr_robot_model_services/GetCameraPose', timeout=5)
        pose = rospy.ServiceProxy('/asr_robot_model_services/GetCameraPose',GetPose)
        rospy.loginfo(pose().pose)
        return pose().pose
    except Exception, e:
        rospy.logwarn("Couldn't get Camera pose from service call.")
        rospy.logwarn(e)
	pass

def detection_callback(data):
    global detected_objects
        # found object for the first time 
    rospy.loginfo("Found " + data.type)
    
    if ( len(data.sampledPoses) == 0 ):
	rospy.logerr("Got a AsrObject without poses.")
	sys.exit(1)
    
    # Construct FoundObject to push to world_model
    # objects are transformed to map frame in world_model
    object = AsrObject()
    object = data
    detected_objects.append(object)


if __name__ == '__main__': 
    global detected_objects
    # Initialize the node and name it.
    rospy.init_node('test_world_model')
 
    sub = rospy.Subscriber('/stereo/objects',AsrObject,detection_callback)
                               
         
    # Push to found objects in world model     
    rate = rospy.Rate(0.3) # 10hz
    while not rospy.is_shutdown():    	
	try:
    		rospy.wait_for_service('/env/asr_world_model/push_found_object_list',timeout=3)
		push_found_objects = rospy.ServiceProxy('/env/asr_world_model/push_found_object_list', PushFoundObjectList)
                push_found_objects(list(detected_objects))
                detected_objects = []
    	except (rospy.exceptions.ROSException, rospy.ServiceException) as e:
		rospy.logwarn("Could not find world_model service")
		sys.exit(1)
	rate.sleep()
