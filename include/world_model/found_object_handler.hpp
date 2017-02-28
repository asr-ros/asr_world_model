/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Hutmacher Robin, Karrenbauer Oliver, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Trautmann Jeremias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#pragma once

// Global Inclusion
#include <boost/shared_ptr.hpp>
#include <vector>
#include <map>
#include <list>
#include <eigen3/Eigen/Geometry>


// ROS Main Inclusion
#include <ros/ros.h>


// ROS-wide Inclusion
#include <asr_msgs/AsrObject.h>
#include <asr_object_database/ObjectMetaData.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>


// Local Inclusion
#include "asr_world_model/GetFoundObjectList.h"
#include "asr_world_model/PushFoundObject.h"
#include "asr_world_model/PushFoundObjectList.h"
#include "asr_world_model/VisualizeSampledPoses.h"

#include "world_model/model/settings.hpp"
#include "world_model/model/model_type.hpp"
#include "world_model/model/model_object.hpp"
#include "world_model/helper/debug_helper.hpp"
#include "world_model/helper/pose_helper.hpp"
#include "world_model/world_model_visualizer_rviz.hpp"

namespace world_model
{


/*!
 * \brief WorldModel class provides services for adding the viewports of the next best views to a list and retrieve them. Additionally, it provides services for adding objects, detected by object localization and retrieve them.
 */
class FoundObjectHandler
{
public:
    /* ----------------- Public members  ------------------  */
    // Wrapped Constants

    // PushFoundObject
    static const inline std::string GetPushFoundObjectServiceName()
    { return "push_found_object"; }

    // PushFoundObjectList
    static const inline std::string GetPushFoundObjectListServiceName()
    { return "push_found_object_list"; }

    // EmptyFoundObjectList
    static const inline std::string GetEmptyFoundObjectListServiceName()
    { return "empty_found_object_list"; }

    // GetFoundObjectList
    static const inline std::string GetGetFoundObjectListServiceName()
    { return "get_found_object_list"; }

    // VisualizeSampledPoses
    static const inline std::string GetVisualizeSampledPosesName()
    { return "visualize_sampled_poses"; }

private:
    const double DEG_TO_RAD = M_PI / 180.0;
    const double EPSILON = 0.0001;

    /* ----------------- Private members  ------------------  */
    DebugHelperPtr debug_helper_ptr_;
    PoseHelperPtr pose_helper_ptr_;
    tf::TransformListener tf_transform_listener_;

    // Services
    WorldModelVisualizerRVIZPtr world_model_visualizer_ptr_;

    // Clients
    boost::shared_ptr<ros::ServiceClient> object_type_client_ptr_;

    ModelTypePtrPerTypeMapPtr model_type_ptr_per_type_map_ptr_;

    SettingsPtr settings_ptr_;

public:
    /* ----------------- Public functions  ------------------  */
    /*!
     * \brief Creates a new instance of FoundObjectHandler
     */
    FoundObjectHandler(boost::shared_ptr<ros::Publisher> marker_publisher_ptr,
                       boost::shared_ptr<ros::ServiceClient> object_type_client_ptr,
                       ModelTypePtrPerTypeMapPtr model_type_ptr_per_type_map_ptr,
                       SettingsPtr settings_ptr);

    /*!
     * \brief Empties the found object list.
     * \param request the associated request object of the service call
     * \param response the associated response object of the service
     * \return the success of the service call.
     */
    bool processEmptyFoundObjectListServiceCall(std_srvs::Empty::Request &request,
                                                std_srvs::Empty::Response &response);

    /*!
     * \brief Returns the already found objects.
     * \param request the associated request object of the service call
     * \param response the associated response object of the service
     * \return the success of the service call.
     */
    bool processGetFoundObjectListServiceCall(asr_world_model::GetFoundObjectList::Request &request,
                                              asr_world_model::GetFoundObjectList::Response &response);
    /*!
     * \brief Pushes the found object to a list.
     * \param request the associated request object of the service call
     * \param response the associated response object of the service
     * \return the success of the service call.
     */
    bool processPushFoundObjectListServiceCall(asr_world_model::PushFoundObjectList::Request &request,
                                               asr_world_model::PushFoundObjectList::Response &response);

    /*!
     * \brief Pushes the found object to a list.
     * \param request the associated request object of the service call
     * \param response the associated response object of the service
     * \return the success of the service call.
     */
    inline bool processPushFoundObjectServiceCall(asr_world_model::PushFoundObject::Request &request,
                                                  asr_world_model::PushFoundObject::Response &response)
    {
        debug_helper_ptr_->write(std::stringstream() << "Calling processPushFoundObjectServiceCall", (DebugHelper::SERVICE_CALLS + DebugHelper::FOUND_OBJECT));
        return pushFoundObject(request.found_object);
    }

    bool processVisualizeSampledPosesCall(asr_world_model::VisualizeSampledPoses::Request &request,
                                          asr_world_model::VisualizeSampledPoses::Response &response);

private:
    /* ----------------- Private functions  ------------------  */

    /*!
     * \brief
     * \param candidate the found object candidate
     * \return the success of the action
     */
    bool pushFoundObject(asr_msgs::AsrObject &candidate);

    void updateBestHypothesis(ModelObjectPtr &currentModelObjectPtr, const asr_msgs::AsrObject &candidate);

    /*!
     * \brief Generates samples for object pose estimation to model uncertainty
     * \param candidate the found object candidate
     * \return success of comunication with object database
     */
    bool sampleFoundObject(asr_msgs::AsrObject &candidate);

    /*!
     * \brief Transform object pose to base frame of world model
     * \param candidate the found object candidate
     */
    bool transformToBaseFrame(asr_msgs::AsrObject &candidate) const;

    /*!
     * \brief Determines if two objects are neighbors to each other in terms of position and orientation distance
     * \param the two objects
     * \return returns true if the two objects are neighbors
     */
    bool evaluateNeighborhood(const asr_msgs::AsrObject &object1, const asr_msgs::AsrObject &object2) const;


    //COMMENT
    void visualizeFoundObjects();

    std::vector<geometry_msgs::Pose> interpolateOrientationAroundAxis(
            const geometry_msgs::Pose &poseToInterpolate, const double interpolationOffset, const Eigen::Vector3d &axisToRotate, const int numberOfInterpolationsPerOrientation) const;

    geometry_msgs::Pose rotateOrientationAroundAxis(const geometry_msgs::Pose &poseToRotate, const double interpolationOffset, const Eigen::Vector3d &axisToRotate) const;

    std::vector<geometry_msgs::Pose> interpolatePoses(const geometry_msgs::Pose &fromPose, const geometry_msgs::Pose &toPose, const int numberOfInterpolations) const;

};


inline bool comparePbdObjectCluster (const PbdObjectCluster& first, const PbdObjectCluster& second)
{
    return ( first.second > second.second );
}

}
