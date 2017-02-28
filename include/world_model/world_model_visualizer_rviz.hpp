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

//Pkg includes
#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <asr_msgs/AsrObject.h>

#include <eigen_conversions/eigen_msg.h>
#include <boost/filesystem.hpp>

namespace world_model
{
class WorldModelVisualizerRVIZ
{
private:
    ros::Publisher publisher_;
    unsigned int marker_id_;
    visualization_msgs::MarkerArray marker_array_;

    inline void addMarker(visualization_msgs::Marker &marker)
    {
        marker.id = marker_id_++;
        marker_array_.markers.push_back(marker);
    }
public:
    WorldModelVisualizerRVIZ(ros::Publisher publisher):
        publisher_(publisher),
        marker_id_(0)
    {}
    inline void publishCollectedMarkers()
    {
        publisher_.publish(marker_array_);
    }
    void clearLastPublication();
    void addFoundObjectVisualization(const asr_msgs::AsrObject &pbd_object, const bool &visualizeSampledPoses);
};
typedef boost::shared_ptr<WorldModelVisualizerRVIZ> WorldModelVisualizerRVIZPtr;
}
