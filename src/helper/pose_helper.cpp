/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Hutmacher Robin, Karrenbauer Oliver, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Trautmann Jeremias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "world_model/helper/pose_helper.hpp"

namespace world_model {

PoseHelperPtr PoseHelper::instance_ptr_;

PoseHelperPtr PoseHelper::getInstance() {
    if (!instance_ptr_) {
        instance_ptr_ = PoseHelperPtr(new PoseHelper());
        resetInstance();
    }
    return instance_ptr_;
}

void PoseHelper::resetInstance() {
    if (instance_ptr_) {
        // Params can be reseted here
    }
}

PoseHelper::PoseHelper() {
}


double PoseHelper::calcDistancePositionEucl(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2) {
    return sqrt(pow(pose1.position.x - pose2.position.x, 2)
                + pow(pose1.position.y - pose2.position.y, 2)
                + pow(pose1.position.z - pose2.position.z, 2));
}

double PoseHelper::calcAngularDistanceInRad(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2) {
    Eigen::Quaterniond rotation1 = convertPoseQuatToQuat(pose1);
    rotation1.normalize();
    Eigen::Quaterniond rotation2 = convertPoseQuatToQuat(pose2);
    rotation2.normalize();
    return rotation1.angularDistance(rotation2);
}

bool PoseHelper::checkPosesAreApproxEquale(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2,
                                           const double position_threshold, const double orientation_rad_threshold) {
    return checkPositionsAreApproxEquale(pose1, pose2, position_threshold)
            && checkOrientationsAreApproxEquale(pose1, pose2, orientation_rad_threshold);
}

bool PoseHelper::checkPositionsAreApproxEquale(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2, const double position_threshold) {
    const double distance_position = calcDistancePositionEucl(pose1, pose2);
    return position_threshold > distance_position;
}

bool PoseHelper::checkOrientationsAreApproxEquale(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2, const double orientation_rad_threshold) {
    const double distance_orientation_rad = calcAngularDistanceInRad(pose1, pose2);
    return orientation_rad_threshold > distance_orientation_rad;
}

}



