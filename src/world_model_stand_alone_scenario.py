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
import random

from geometry_msgs.msg import (Pose, PoseWithCovariance, PoseWithCovarianceStamped, Point, Quaternion, Twist, Vector3)
from asr_msgs.msg import AsrObject
from actionlib import *
from actionlib.msg import *
from next_best_view.msg import RobotStateMessage
from next_best_view.srv import CalculateCameraPose, TriggerFrustumVisualization, GetPose
from sensor_msgs.msg import JointState
from asr_world_model.srv import PushFoundObject, PushFoundObjectList, GetFoundObjectList, EmptyViewportList

def push_object_in_wm(detected_objects):
    try:
            rospy.wait_for_service('/env/asr_world_model/push_found_object_list',timeout=3)
            push_found_objects = rospy.ServiceProxy('/env/asr_world_model/push_found_object_list', PushFoundObjectList)
            push_found_objects(detected_objects)
    except (rospy.exceptions.ROSException, rospy.ServiceException) as e:
            rospy.logwarn("Could not find world_model service")
            sys.exit(1)

def found_object_list():
    try:
            rospy.wait_for_service('/env/asr_world_model/get_found_object_list',timeout=3)
            get_found_objects = rospy.ServiceProxy('/env/asr_world_model/get_found_object_list', GetFoundObjectList)
            print get_found_objects()
    except (rospy.exceptions.ROSException, rospy.ServiceException) as e:
            rospy.logwarn("Could not find world_model service")
            sys.exit(1)

def clear_world_model():
    try:
            rospy.wait_for_service('/env/asr_world_model/empty_viewport_list',timeout=3)
            get_found_objects = rospy.ServiceProxy('/env/asr_world_model/empty_viewport_list', EmptyViewportList)
            get_found_objects()
    except (rospy.exceptions.ROSException, rospy.ServiceException) as e:
            rospy.logwarn("Could not find world_model service")
            sys.exit(1)


def get_coffee_box():
    obj = AsrObject()
    obj.type = 'CoffeeBox'
    obj.header.frame_id = '/camera_left_frame'
    obj.identifier = '0'
    obj.providedBy = 'textured'
    obj.colorName = 'textured'
    obj.meshResourcePath = 'package://asr_object_database/rsc/databases/textured_objects/CoffeeBox/CoffeeBox.dae'
    pose_cov = PoseWithCovariance()
    pose_cov.pose.position.x = random.gauss(0,0.01)
    pose_cov.pose.position.y = random.gauss(0,0.01) + random.randint(0, 1) 
    pose_cov.pose.position.z = 1.12
    if random.randint(0, 5) == 0:
      pose_cov.pose.position.x = pose_cov.pose.position.x + 1
    q = tf.transformations.quaternion_from_euler(90, 0, 0, 'rzyx')
    pose_cov.pose.orientation = Quaternion(*q)
    obj.sampledPoses.append(pose_cov)
    return obj

def get_smacks():
    obj = AsrObject()
    obj.type = 'Smacks'
    obj.header.frame_id = '/camera_left_frame'
    obj.identifier = '0'
    obj.providedBy = 'textured'
    obj.colorName = 'textured'
    obj.meshResourcePath = 'package://asr_object_database/rsc/databases/textured_objects/Smacks/Smacks.dae'
    pose_cov = PoseWithCovariance()
    pose_cov.pose.position.x = random.gauss(0,0.5)
    pose_cov.pose.position.y = random.gauss(0,0.5)
    pose_cov.pose.position.z = 1.12
    q = tf.transformations.quaternion_from_euler(90, 0, 0, 'rzyx')
    pose_cov.pose.orientation = Quaternion(*q)
    obj.sampledPoses.append(pose_cov)
    return obj

if __name__ == '__main__': 
    global detected_objects
    # Initialize the node and name it.
    rospy.init_node('test_world_model')
    object_list = []
    for i in range(500):
        found_object = AsrObject()
        found_object = get_coffee_box()
        object_list.append(found_object)
    push_object_in_wm(object_list)
    found_object_list()
    for i in range(500):
        found_object = AsrObject()
        found_object = get_coffee_box()
        object_list.append(found_object)
    push_object_in_wm(object_list)
    found_object_list()
    clear_world_model()
    found_object_list()
