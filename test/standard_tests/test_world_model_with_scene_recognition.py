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

PKG = 'asr_world_model'

import unittest
import rospy
from asr_msgs.msg import AsrObject
from asr_world_model.srv import PushFoundObject, PushFoundObjectRequest, GetFoundObjectList
from recognizer_prediction_ism.srv import FindScenes, FindScenesRequest, GetPointCloud
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, PoseWithCovariance

class WorldModelTest(unittest.TestCase):

    def setUp(self):
        rospy.init_node("test_world_model", anonymous=True)

        try:
            rospy.wait_for_service('/env/asr_world_model/push_found_object', timeout=5)
            rospy.wait_for_service('/env/asr_world_model/empty_found_object_list', timeout=5)
            rospy.wait_for_service('/env/asr_world_model/env/asr_world_model/get_found_object_list', timeout=5)
            rospy.wait_for_service('/asr_object_database/object_type', timeout=5)
        except rospy.exceptions.ROSException, e:
            rospy.loginfo('Could not reach service' + e.message)

    def test_world_model_with_1_object(self):

        world_model_reset = rospy.ServiceProxy('/env/asr_world_model/empty_found_object_list', Empty)
        world_model_reset()

        req = PushFoundObjectRequest()
        detected_pbd_object = AsrObject()
        detected_pbd_object.type = 'CoffeeBox'
        detected_pbd_object.header.frame_id = '/camera_left_frame'
        detected_pbd_object.identifier = ''
        detected_pbd_object.providedBy = 'textured'
        detected_pbd_object.colorName = 'textured'
        detected_pbd_object.meshResourcePath = 'package://asr_object_database/rsc/databases/textured_objects/CoffeeBox/CoffeeBox.dae'

        detected_pose = Pose()
        detected_pose.position.x = 1
        detected_pose.position.y = 2
        detected_pose.position.z = 3
        detected_pose.orientation.w = 0.695641823146944
        detected_pose.orientation.x = -0.702895791692416
        detected_pose.orientation.y = -0.102411625776331
        detected_pose.orientation.z = 0.107386306462868

        detected_pose_with_c = PoseWithCovariance()
        detected_pose_with_c.pose = detected_pose

        detected_pbd_object.sampledPoses.append(detected_pose_with_c)

        push_found_object = rospy.ServiceProxy('/env/asr_world_model/push_found_object', PushFoundObject)
        push_found_object(detected_pbd_object)

        get_found_objects = rospy.ServiceProxy('/env/asr_world_model/get_found_object_list', GetFoundObjectList)
        resp = get_found_objects()
        self.assertEqual(len(resp.object_list), 1)

        for found_object in resp.object_list:
            self.assertEqual(len(found_object.sampledPoses), 1)


        find_scenes = rospy.ServiceProxy('/rp_ism_node/find_scenes', FindScenes)
        find_scenes_req = FindScenesRequest()
        for pbd_object in resp.object_list:
            find_scenes_req.objects.append(pbd_object)
        find_scenes(find_scenes_req)

        rp_ism_node_predict = rospy.ServiceProxy('/rp_ism_node/get_point_cloud', GetPointCloud)
        rp_ism_node_predict()


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_world_model', WorldModelTest)
