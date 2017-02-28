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

#include <ros/ros.h>
#include <math.h>
#include <boost/shared_ptr.hpp>
#include <eigen3/Eigen/Geometry>

#include "geometry_msgs/Pose.h"

namespace world_model {

class PoseHelper {

private:
    static boost::shared_ptr<PoseHelper> instance_ptr_;

    Eigen::Quaterniond convertPoseQuatToQuat(const geometry_msgs::Pose &pose) {
        return Eigen::Quaterniond(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    }

    PoseHelper();

public:

    static boost::shared_ptr<PoseHelper> getInstance();
    static void resetInstance();

    double calcDistancePositionEucl(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2);
    double calcAngularDistanceInRad(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2);

    bool checkPosesAreApproxEquale(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2, const double position_threshold, const double orientation_rad_threshold);
    bool checkPositionsAreApproxEquale(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2, const double position_threshold);
    bool checkOrientationsAreApproxEquale(const geometry_msgs::Pose &pose1, const geometry_msgs::Pose &pose2, const double orientation_rad_threshold);

};

typedef boost::shared_ptr<PoseHelper> PoseHelperPtr;

}







