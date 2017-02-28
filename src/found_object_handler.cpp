/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Hutmacher Robin, Karrenbauer Oliver, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Trautmann Jeremias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "world_model/found_object_handler.hpp"

namespace world_model
{

FoundObjectHandler::FoundObjectHandler(boost::shared_ptr<ros::Publisher> marker_publisher_ptr,
                                       boost::shared_ptr<ros::ServiceClient> object_type_client_ptr,
                                       ModelTypePtrPerTypeMapPtr model_type_ptr_per_type_map_ptr,
                                       SettingsPtr settings_ptr) :
    tf_transform_listener_(),
    object_type_client_ptr_(object_type_client_ptr),
    model_type_ptr_per_type_map_ptr_(model_type_ptr_per_type_map_ptr),
    settings_ptr_(settings_ptr)
{
    debug_helper_ptr_ = DebugHelper::getInstance();
    pose_helper_ptr_ = PoseHelper::getInstance();
    world_model_visualizer_ptr_ = WorldModelVisualizerRVIZPtr(new WorldModelVisualizerRVIZ(*marker_publisher_ptr));
    ROS_INFO_STREAM("FoundObjectHandler initialized");
}

bool FoundObjectHandler::processPushFoundObjectListServiceCall(asr_world_model::PushFoundObjectList::Request &request,
                                                               asr_world_model::PushFoundObjectList::Response &response)
{
    debug_helper_ptr_->write(std::stringstream() << "Calling processPushFoundObjectListServiceCall with " << request.found_object_list.size() << " estimates in list", (DebugHelper::SERVICE_CALLS + DebugHelper::FOUND_OBJECT));
    bool result = true;
    for (asr_msgs::AsrObject &foundObject : request.found_object_list)
    {
        result &= pushFoundObject(foundObject);
        if(!result) ROS_ERROR_STREAM("Pushing found object to world model failed for type/id: " << foundObject.type << "/" << foundObject.identifier);
    }
    return result;
}

bool FoundObjectHandler::pushFoundObject(asr_msgs::AsrObject &candidate)
{
    const std::string &CANDIDATE_TYPE = candidate.type;
    const std::string &CANDIDATE_IDENTIFIER = candidate.identifier;

    //Check if given candidate is known
    ModelTypePtrPerTypeMap::iterator type_it = model_type_ptr_per_type_map_ptr_->find(CANDIDATE_TYPE);
    if (type_it == model_type_ptr_per_type_map_ptr_->end()) {
        ROS_WARN_STREAM("The given type was not in the ISM_Tables -> not all information are available for it: " << CANDIDATE_TYPE);
        type_it = model_type_ptr_per_type_map_ptr_->insert(std::make_pair(CANDIDATE_TYPE, ModelTypePtr(new ModelType(CANDIDATE_TYPE)))).first;
    }

    ModelObjectPtrPerIdMap::iterator id_it = type_it->second->model_object_ptr_per_id_map.find(CANDIDATE_IDENTIFIER);
    if (id_it == type_it->second->model_object_ptr_per_id_map.end()) {
        ROS_WARN_STREAM("The given type and id was not in the ISM_Tables -> not all information are available for it: type: " << CANDIDATE_TYPE << ", id: " << CANDIDATE_IDENTIFIER);
        id_it = type_it->second->model_object_ptr_per_id_map.insert(std::make_pair(CANDIDATE_IDENTIFIER, ModelObjectPtr(new ModelObject(CANDIDATE_IDENTIFIER)))).first;
    }

    if (!transformToBaseFrame(candidate)) {
        return false;
    }

    if (!sampleFoundObject(candidate)) {
        return false;
    }

    ModelObjectPtr &currentModelObjectPtr = id_it->second;
    //In the past, this was just the best cluster for the object being currently pushed.
    //Since multiple objects of the same type might be present in world_model now, replaced best cluster with set of best clusters (one for each object of same type and id).

    if (currentModelObjectPtr->bestHypotheses.size() == 0) {
        //Create list for clusters of similiar object poses for given object type id
        currentModelObjectPtr->allFoundHypotheses.push_back(std::make_pair(candidate, 1));
        //Create first cluster in best_hypothesis for this pushed combination of type and id.
        currentModelObjectPtr->bestHypotheses.push_back(std::make_pair(candidate, 1));

        debug_helper_ptr_->write(std::stringstream() << "Received object hypothesis for " << CANDIDATE_TYPE << " with ID " << CANDIDATE_IDENTIFIER << " is the first. Is trivially best hypothesis", DebugHelper::FOUND_OBJECT);
        debug_helper_ptr_->write(std::stringstream() << "object_rating_min_count_ = " << settings_ptr_->object_rating_min_count << " ignored for first and only cluster", DebugHelper::FOUND_OBJECT);
    } else {
        updateBestHypothesis(currentModelObjectPtr, candidate);
    }

    const geometry_msgs::Point &candidate_position = candidate.sampledPoses.front().pose.position;
    debug_helper_ptr_->write(std::stringstream() << "Position of object estimate, having been added to cluster list: " << candidate_position.x << ", " << candidate_position.y << ", " << candidate_position.z, DebugHelper::FOUND_OBJECT);

    visualizeFoundObjects();
    return true;
}

void FoundObjectHandler::updateBestHypothesis(ModelObjectPtr &currentModelObjectPtr, const asr_msgs::AsrObject &candidate) {
    PbdObjectClusterList currentBestHypothesis;
    //If other objects already exist for this id, update the neighbor counter for potential neighbors and update the new best hypothesis
    PbdObjectClusterList &allFoundHypotheses = currentModelObjectPtr->allFoundHypotheses;

    debug_helper_ptr_->write(std::stringstream() << "Received object hypothesis for " << candidate.type << " with ID " << candidate.identifier << " is NOT the first", DebugHelper::FOUND_OBJECT);
    debug_helper_ptr_->write(std::stringstream() << "There are already " << allFoundHypotheses.size() << " clusters for that object", DebugHelper::FOUND_OBJECT);

    int neighborCount = 1;

    //Increment all sizes of clusters, neighbouring to currently pushed estimate.
    for (PbdObjectCluster &pdbObjectCluster : allFoundHypotheses)
    {
        if (evaluateNeighborhood(pdbObjectCluster.first, candidate))
        {
            ++pdbObjectCluster.second;
            ++neighborCount;
        }
    }
    //Insert pushed object estimate into cluster list.
    allFoundHypotheses.push_back(std::make_pair(candidate, neighborCount));

    debug_helper_ptr_->write(std::stringstream() << "Added new cluster for " << candidate.type << " with ID " << candidate.identifier << " with neighborCount " << neighborCount << " into clusterlist", DebugHelper::FOUND_OBJECT);
    debug_helper_ptr_->write(std::stringstream() << "New cluster list size is " << allFoundHypotheses.size(), DebugHelper::FOUND_OBJECT);


    //Sorting list of best clusters for currently pushed estimate.
    allFoundHypotheses.sort(comparePbdObjectCluster);


    //======== CREATE BEST CLUSTER LIST FOR PUSHED COMBINATION OF TYPE AND ID. ========

    //Iterate over the sorted cluster list for the pushed estimate.
    for (const PbdObjectCluster &pdbObjectCurrentCluster : allFoundHypotheses)
    {
        //Fill new current_best_hypotheses list with currentClusterList.
        //Filter out all clusters in currentClusterList that are either redundant to clusters in current_best_hypotheses or that do not have enough elements (object_rating_min_count_).
        bool isInNeighborhood = false;

        const geometry_msgs::Pose &clusterListCenterPose = pdbObjectCurrentCluster.first.sampledPoses.front().pose;
        for (const PbdObjectCluster &pdbObjectCurrentBestHypothese : currentBestHypothesis)
        {
            //Redundancy test
            const geometry_msgs::Pose &bestCenterPose  = pdbObjectCurrentBestHypothese.first.sampledPoses.front().pose;
            isInNeighborhood |= pose_helper_ptr_->checkPosesAreApproxEquale(clusterListCenterPose, bestCenterPose, 0.1, 2.0*M_PI);
        }

        if (!isInNeighborhood && pdbObjectCurrentCluster.second >= settings_ptr_->object_rating_min_count)
        {
            currentBestHypothesis.push_back(std::make_pair(pdbObjectCurrentCluster.first, pdbObjectCurrentCluster.second));
            debug_helper_ptr_->write(std::stringstream() << "Added cluster to best cluster list (Neighborhood count: " << pdbObjectCurrentCluster.second << ")", DebugHelper::FOUND_OBJECT);
        }
    }

    debug_helper_ptr_->write(std::stringstream() << "Created new best cluster list with " << currentBestHypothesis.size() << " elements (objects) for " << candidate.type << " with ID " << candidate.identifier , DebugHelper::FOUND_OBJECT);

    //======= REPLACE BEST CLUSTER LIST FOR CURRENTLY PUSHED ESTIMATE IN MAP OF BEST CLUSTERS FOR ALL TYPES AND IDS =======
    currentModelObjectPtr->bestHypotheses = currentBestHypothesis;
}

bool FoundObjectHandler::evaluateNeighborhood(const asr_msgs::AsrObject &object1, const asr_msgs::AsrObject &object2) const {
    const geometry_msgs::Pose &detected_pose1 = object1.sampledPoses.front().pose;
    const geometry_msgs::Pose &detected_pose2 = object2.sampledPoses.front().pose;
    return pose_helper_ptr_->checkPosesAreApproxEquale(detected_pose1, detected_pose2, settings_ptr_->object_position_distance_threshold,
                                                       settings_ptr_->object_orientation_rad_distance_threshold);
}

bool FoundObjectHandler::transformToBaseFrame(asr_msgs::AsrObject &candidate) const {
    geometry_msgs::PoseStamped cameraPose, mapPose;
    cameraPose.header = candidate.header;
    cameraPose.pose = candidate.sampledPoses.front().pose;
    //transform to map frame

    try {
        tf::StampedTransform transform;
        tf_transform_listener_.lookupTransform(candidate.header.frame_id, "map", ros::Time(0), transform);
        //Do not do this if objects are not static. Required for simulation.
        cameraPose.header.stamp = transform.stamp_;
        tf_transform_listener_.transformPose("map", cameraPose, mapPose);

    } catch (tf::TransformException ex) {
        ROS_ERROR_STREAM("tf exception when converting object to map frame :: " << ex.what());
        return false;
    }

    candidate.header.frame_id = mapPose.header.frame_id;
    geometry_msgs::PoseWithCovariance pose_with_c;
    pose_with_c.pose = mapPose.pose;
    candidate.sampledPoses.clear();
    candidate.sampledPoses.push_back(pose_with_c);
    return true;
}

bool FoundObjectHandler::sampleFoundObject(asr_msgs::AsrObject &candidate) {
    if (!settings_ptr_->enable_object_sampling) {
        debug_helper_ptr_->write(std::stringstream() << "Sample found objects disabled", DebugHelper::FOUND_OBJECT);
        return true;
    }
    debug_helper_ptr_->write(std::stringstream() << "Entering sampleFoundObject()", DebugHelper::FOUND_OBJECT);

    if (!settings_ptr_->objects_to_sample.empty()) {
        bool isObjectToSample = false;
        for (const std::pair<std::string, std::string> &type_to_id : settings_ptr_->objects_to_sample) {
            if (type_to_id.first == candidate.type && type_to_id.second == candidate.identifier) {
                isObjectToSample = true;
                break;
            }
        }
        if (!isObjectToSample) {
            ROS_WARN_STREAM("Given object is disabled for sampling type: " << candidate.type << " id: " << candidate.identifier);
            return true;
        }
    }

    double X_AXIS_ERROR;
    double Y_AXIS_ERROR;
    double Z_AXIS_ERROR;

    double ALPHA_ERROR;
    double BETA_ERROR;
    double GAMMA_ERROR;

    if (settings_ptr_->calculate_deviations) {
        X_AXIS_ERROR   = sqrt(3.0)/2.0 * settings_ptr_->bin_size;
        Y_AXIS_ERROR   = sqrt(3.0)/2.0 * settings_ptr_->bin_size;
        Z_AXIS_ERROR   = sqrt(3.0)/2.0 * settings_ptr_->bin_size;

        ALPHA_ERROR    = settings_ptr_->maxProjectionAngleDeviation;
        BETA_ERROR     = settings_ptr_->maxProjectionAngleDeviation;
        GAMMA_ERROR    = settings_ptr_->maxProjectionAngleDeviation;
    } else {
        const std::string &recognizerName = (*model_type_ptr_per_type_map_ptr_)[candidate.type]->recognizerName;
        if (recognizerName.size() == 0) {
            ROS_ERROR_STREAM("RecognizerName for type/id" << candidate.type << "/" << candidate.identifier << " not found: Please check the ISM_Tabel.");
            return false;
        }

        asr_object_database::ObjectMetaData srv;
        srv.request.object_type = candidate.type;
        srv.request.recognizer = recognizerName;

        if (object_type_client_ptr_->call(srv)) {
            std::vector<double> &deviations = srv.response.deviations;
            if (deviations.size() == 6) {
                std::vector<double>::iterator deviations_it = deviations.begin();

                debug_helper_ptr_->write(std::stringstream() << "Sampling object (type = " << candidate.type <<
                                         ", id = " << candidate.identifier << ") with errors in", DebugHelper::FOUND_OBJECT);
                X_AXIS_ERROR   = *(deviations_it++);
                Y_AXIS_ERROR   = *(deviations_it++);
                Z_AXIS_ERROR   = *(deviations_it++);

                ALPHA_ERROR    = *(deviations_it++);
                BETA_ERROR     = *(deviations_it++);
                GAMMA_ERROR    = *(deviations_it++);
            } else {
                ROS_ERROR_STREAM("The deviations from the objectdatabase has not 6 entries but " << deviations.size());
                return false;
            }
        } else {
            ROS_ERROR("Could not call the asr_object_database::ObjectMetaData service call");
            return false;
        }
    }

    debug_helper_ptr_->write(std::stringstream() << "position.x [m] = " << X_AXIS_ERROR, DebugHelper::FOUND_OBJECT);
    debug_helper_ptr_->write(std::stringstream() << "position.y [m] = " << Y_AXIS_ERROR, DebugHelper::FOUND_OBJECT);
    debug_helper_ptr_->write(std::stringstream() << "position.z [m] = " << Z_AXIS_ERROR, DebugHelper::FOUND_OBJECT);
    debug_helper_ptr_->write(std::stringstream() << "orientation.alpha [deg] = " << ALPHA_ERROR, DebugHelper::FOUND_OBJECT);
    debug_helper_ptr_->write(std::stringstream() << "orientation.beta  [deg] = " << BETA_ERROR, DebugHelper::FOUND_OBJECT);
    debug_helper_ptr_->write(std::stringstream() << "orientation.gamma [deg] = " << GAMMA_ERROR, DebugHelper::FOUND_OBJECT);

    if (!candidate.sampledPoses.size()) {
        ROS_ERROR_STREAM("Got a AsrObject without poses.");
        return false;
    }

    //Getting access to original pose estimation that resides in world model anyway.
    const geometry_msgs::PoseWithCovariance detected_pose_with_c = candidate.sampledPoses.front();
    const geometry_msgs::Pose &detected_pose = detected_pose_with_c.pose;

    debug_helper_ptr_->write(std::stringstream() << "Sampling around that position: x: "
                             << detected_pose.position.x << ", y: " << detected_pose.position.y << ", z: " << detected_pose.position.z, DebugHelper::FOUND_OBJECT);

    // *2, because we interpolate in + and - axis; +1, because we don't want the orginal pose to get lost
    const int numberOfInterpolationsPerPosition = 2 * settings_ptr_->deviation_number_of_samples_position + 1;
    const int numberOfInterpolationsPerOrientation = 2 * settings_ptr_->deviation_number_of_samples_orientation + 1;


    //Sampling additional object poses for recognition result to consider pose uncertainty.
    geometry_msgs::Pose interpolatedPoseMinX = detected_pose;
    interpolatedPoseMinX.position.x -= X_AXIS_ERROR;
    geometry_msgs::Pose interpolatedPoseMaxX = detected_pose;
    interpolatedPoseMaxX.position.x += X_AXIS_ERROR;
    std::vector<geometry_msgs::Pose> allInterpolatedPerX = interpolatePoses(interpolatedPoseMinX, interpolatedPoseMaxX, numberOfInterpolationsPerPosition);

    for (const geometry_msgs::Pose &interpolatedPerX : allInterpolatedPerX) {

        geometry_msgs::Pose interpolatedPoseMinY = interpolatedPerX;
        interpolatedPoseMinY.position.y -= Y_AXIS_ERROR;
        geometry_msgs::Pose interpolatedPoseMaxY = interpolatedPerX;
        interpolatedPoseMaxY.position.y += Y_AXIS_ERROR;
        std::vector<geometry_msgs::Pose> allInterpolatedPerY = interpolatePoses(interpolatedPoseMinY, interpolatedPoseMaxY, numberOfInterpolationsPerPosition);

        for (const geometry_msgs::Pose &interpolatedPerY : allInterpolatedPerY) {

            geometry_msgs::Pose interpolatedPoseMinZ = interpolatedPerY;
            interpolatedPoseMinZ.position.z -= Z_AXIS_ERROR;
            geometry_msgs::Pose interpolatedPoseMaxZ = interpolatedPerY;
            interpolatedPoseMaxZ.position.z += Z_AXIS_ERROR;
            std::vector<geometry_msgs::Pose> allInterpolatedPerZ = interpolatePoses(interpolatedPoseMinZ, interpolatedPoseMaxZ, numberOfInterpolationsPerPosition);

            for (const geometry_msgs::Pose &interpolatedPerZ : allInterpolatedPerZ) {

                std::vector<geometry_msgs::Pose> allInterpolatedPerAlpha =
                        interpolateOrientationAroundAxis(interpolatedPerZ, ALPHA_ERROR, Eigen::Vector3d::UnitX(), numberOfInterpolationsPerOrientation);

                for (const geometry_msgs::Pose &interpolatedPerAlpha : allInterpolatedPerAlpha) {

                    std::vector<geometry_msgs::Pose> allInterpolatedPerBeta =
                            interpolateOrientationAroundAxis(interpolatedPerAlpha, BETA_ERROR, Eigen::Vector3d::UnitY(), numberOfInterpolationsPerOrientation);

                    for (const geometry_msgs::Pose &interpolatedPerGamma : allInterpolatedPerBeta) {

                        std::vector<geometry_msgs::Pose> allInterpolatedPerGamma =
                                interpolateOrientationAroundAxis(interpolatedPerGamma, GAMMA_ERROR, Eigen::Vector3d::UnitZ(), numberOfInterpolationsPerOrientation);

                        for (const geometry_msgs::Pose &interpolatedPerGamma : allInterpolatedPerGamma) {

                            double positionDistance = pose_helper_ptr_->calcDistancePositionEucl(interpolatedPerGamma, detected_pose);
                            double orientationDistance = fabs(pose_helper_ptr_->calcAngularDistanceInRad(interpolatedPerGamma, detected_pose));

                            if (positionDistance <= EPSILON && orientationDistance <= EPSILON) {
                                // it is the same pose as the recorded one
                                continue;
                            }

                            if (settings_ptr_->calculate_deviations) {
                                if (positionDistance - (sqrt(3.0)/2.0 * settings_ptr_->bin_size) >= EPSILON
                                        || orientationDistance - (settings_ptr_->maxProjectionAngleDeviation * DEG_TO_RAD) >= EPSILON) {
                                    continue;
                                }
                            }

                            geometry_msgs::PoseWithCovariance current_pose_with_covariance;
                            current_pose_with_covariance.pose = interpolatedPerGamma;
                            current_pose_with_covariance.covariance = detected_pose_with_c.covariance;
                            candidate.sampledPoses.push_back(current_pose_with_covariance);
                        }
                    }
                }
            }
        }
    }

    ROS_INFO_STREAM("Number of sampled poses: " << candidate.sampledPoses.size() - 1);
    return true;
}

std::vector<geometry_msgs::Pose> FoundObjectHandler::interpolateOrientationAroundAxis(
        const geometry_msgs::Pose &poseToInterpolate, const double interpolationOffset, const Eigen::Vector3d &axisToRotate, const int numberOfInterpolationsPerOrientation) const {

    const geometry_msgs::Pose &interpolatedPoseMin = rotateOrientationAroundAxis(poseToInterpolate, interpolationOffset, axisToRotate);
    const geometry_msgs::Pose &interpolatedPoseMax = rotateOrientationAroundAxis(poseToInterpolate, -1.0 * interpolationOffset, axisToRotate);

    return interpolatePoses(interpolatedPoseMin, interpolatedPoseMax, numberOfInterpolationsPerOrientation);
}

geometry_msgs::Pose FoundObjectHandler::rotateOrientationAroundAxis(
        const geometry_msgs::Pose &poseToRotate, const double interpolationOffset, const Eigen::Vector3d &axisToRotate) const {

    Eigen::Quaterniond poseToRotateQuat(poseToRotate.orientation.w,
                                        poseToRotate.orientation.x,
                                        poseToRotate.orientation.y,
                                        poseToRotate.orientation.z);
    poseToRotateQuat.normalize();

    Eigen::Matrix3d rotatedOffsetMat;
    rotatedOffsetMat = Eigen::AngleAxisd(interpolationOffset * DEG_TO_RAD, axisToRotate);
    Eigen::Quaterniond rotatedOffsetQuat(rotatedOffsetMat);
    rotatedOffsetQuat.normalize();

    Eigen::Quaterniond rotatedPoseQuat = rotatedOffsetQuat * poseToRotateQuat;

    geometry_msgs::Pose rotatedPose = poseToRotate;
    rotatedPose.orientation.w = rotatedPoseQuat.w();
    rotatedPose.orientation.x = rotatedPoseQuat.x();
    rotatedPose.orientation.y = rotatedPoseQuat.y();
    rotatedPose.orientation.z = rotatedPoseQuat.z();
    return rotatedPose;
}

std::vector<geometry_msgs::Pose> FoundObjectHandler::interpolatePoses(const geometry_msgs::Pose &fromPose, const geometry_msgs::Pose &toPose, const int numberOfInterpolations) const {
    std::vector<geometry_msgs::Pose> allInterpolatedPoses;

    Eigen::Vector3d fromPosition(fromPose.position.x, fromPose.position.y, fromPose.position.z);
    Eigen::Vector3d toPosition(toPose.position.x, toPose.position.y, toPose.position.z);
    Eigen::Quaterniond fromOrient(fromPose.orientation.w,
                                  fromPose.orientation.x,
                                  fromPose.orientation.y,
                                  fromPose.orientation.z);
    Eigen::Quaterniond toOrient(toPose.orientation.w,
                                toPose.orientation.x,
                                toPose.orientation.y,
                                toPose.orientation.z);

    double divisor = static_cast<double>(numberOfInterpolations + 1);
    for (int i = 1; i <= numberOfInterpolations; ++i) {
        double weightForFirst = (divisor - i) / divisor;
        double weightForSecond = i / divisor;

        Eigen::Vector3d currentInterpolatedPosition = fromPosition * weightForFirst + toPosition * weightForSecond;
        Eigen::Quaterniond currentInterpolatedQuat = fromOrient.slerp(weightForSecond, toOrient);

        geometry_msgs::Pose currentInterpolatedPose;
        currentInterpolatedPose.position.x = currentInterpolatedPosition[0];
        currentInterpolatedPose.position.y = currentInterpolatedPosition[1];
        currentInterpolatedPose.position.z = currentInterpolatedPosition[2];
        currentInterpolatedPose.orientation.w = currentInterpolatedQuat.w();
        currentInterpolatedPose.orientation.x = currentInterpolatedQuat.x();
        currentInterpolatedPose.orientation.y = currentInterpolatedQuat.y();
        currentInterpolatedPose.orientation.z = currentInterpolatedQuat.z();

        bool isEquale = false;
        for (const geometry_msgs::Pose &interpolatedPose : allInterpolatedPoses) {
            if (pose_helper_ptr_->checkPosesAreApproxEquale(interpolatedPose, currentInterpolatedPose, EPSILON, EPSILON)) {
                isEquale = true;
                break;
            }
        }
        if (!isEquale) {
            allInterpolatedPoses.push_back(currentInterpolatedPose);
        }
    }
    return allInterpolatedPoses;
}

void FoundObjectHandler::visualizeFoundObjects() {
    world_model_visualizer_ptr_->clearLastPublication();
    for (const std::pair<std::string, ModelTypePtr> &model_type_ptr_per_type_pair: *model_type_ptr_per_type_map_ptr_) {
        for (const std::pair<std::string, ModelObjectPtr> &model_object_ptr_per_id_pair : model_type_ptr_per_type_pair.second->model_object_ptr_per_id_map) {
            for (const PbdObjectCluster &best_object_hypotheses : model_object_ptr_per_id_pair.second->bestHypotheses) {
                world_model_visualizer_ptr_->addFoundObjectVisualization(best_object_hypotheses.first, false);
            }
        }
    }
    world_model_visualizer_ptr_->publishCollectedMarkers();
}

bool FoundObjectHandler::processVisualizeSampledPosesCall(asr_world_model::VisualizeSampledPoses::Request &request,
                                                          asr_world_model::VisualizeSampledPoses::Response &response) {
    debug_helper_ptr_->write(std::stringstream() << "Calling processVisualizeSampledPosesCall for object_type: "
                             << request.object_type << " object_id: " << request.object_id, (DebugHelper::SERVICE_CALLS + DebugHelper::FOUND_OBJECT));

    //Check if given candidate is known
    ModelTypePtrPerTypeMap::iterator type_it = model_type_ptr_per_type_map_ptr_->find(request.object_type);
    if (type_it == model_type_ptr_per_type_map_ptr_->end()) {
        ROS_WARN_STREAM("The given type was not in the ISM_Tables -> processVisualizeSampledPosesCall not possible: " << request.object_type);
        return false;
    }

    ModelObjectPtrPerIdMap::iterator id_it = type_it->second->model_object_ptr_per_id_map.find(request.object_id);
    if (id_it == type_it->second->model_object_ptr_per_id_map.end()) {
        ROS_WARN_STREAM("The given type and id was not in the ISM_Tables -> processVisualizeSampledPosesCall not possible: type: " << request.object_type << ", id: " << request.object_id);
        return false;
    }

    const ModelObjectPtr &currentModelObjectPtr = id_it->second;

    for (const PbdObjectCluster &best_object_hypotheses : currentModelObjectPtr->bestHypotheses) {
        world_model_visualizer_ptr_->addFoundObjectVisualization(best_object_hypotheses.first, true);
    }

    world_model_visualizer_ptr_->publishCollectedMarkers();

    return true;
}


bool FoundObjectHandler::processEmptyFoundObjectListServiceCall(std_srvs::Empty::Request &request,
                                                                std_srvs::Empty::Response &response)
{
    debug_helper_ptr_->write(std::stringstream() << "Calling processEmptyFoundObjectListServiceCall", (DebugHelper::SERVICE_CALLS + DebugHelper::FOUND_OBJECT));
    for (const std::pair<std::string, ModelTypePtr> &model_type_ptr_per_type_pair: *model_type_ptr_per_type_map_ptr_) {
        for (const std::pair<std::string, ModelObjectPtr> &model_object_ptr_per_id_pair : model_type_ptr_per_type_pair.second->model_object_ptr_per_id_map) {
            model_object_ptr_per_id_pair.second->allFoundHypotheses.clear();
            model_object_ptr_per_id_pair.second->bestHypotheses.clear();
        }
    }

    world_model_visualizer_ptr_->clearLastPublication();

    return true;
}

bool FoundObjectHandler::processGetFoundObjectListServiceCall(asr_world_model::GetFoundObjectList::Request &request,
                                                              asr_world_model::GetFoundObjectList::Response &response)
{
    debug_helper_ptr_->write(std::stringstream() << "Calling processGetFoundObjectListServiceCall", (DebugHelper::SERVICE_CALLS + DebugHelper::FOUND_OBJECT));

    unsigned int hypotheses_counter = 0;

    for (const std::pair<std::string, ModelTypePtr> &model_type_ptr_per_type_pair: *model_type_ptr_per_type_map_ptr_) {
        for (const std::pair<std::string, ModelObjectPtr> &model_object_ptr_per_id_pair : model_type_ptr_per_type_pair.second->model_object_ptr_per_id_map) {
            for (const PbdObjectCluster &best_object_hypotheses : model_object_ptr_per_id_pair.second->bestHypotheses) {
                response.object_list.push_back(best_object_hypotheses.first);
                ++hypotheses_counter;
            }
        }
    }

    debug_helper_ptr_->write(std::stringstream() << "There are currently " << hypotheses_counter << " best hypotheses in the world_model", DebugHelper::FOUND_OBJECT);
    return true;
}

}

