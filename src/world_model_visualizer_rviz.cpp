/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Hutmacher Robin, Karrenbauer Oliver, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Trautmann Jeremias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "world_model/world_model_visualizer_rviz.hpp"

namespace world_model
{

void WorldModelVisualizerRVIZ::clearLastPublication()
{
    for (visualization_msgs::Marker &marker: marker_array_.markers) {
        marker.action = visualization_msgs::Marker::DELETE;
    }
    publisher_.publish(marker_array_);
    marker_array_.markers.clear();
    marker_id_ = 0;
}

void WorldModelVisualizerRVIZ::addFoundObjectVisualization(const asr_msgs::AsrObject &pbd_object, const bool &visualizeSampledPoses)
{
    visualization_msgs::Marker marker;
    marker.header = pbd_object.header;

    if (pbd_object.type == "CompactObject")
    {
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.scale.x =  marker.scale.y = marker.scale.z = 0.01;
    }
    else
    {
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        if (pbd_object.colorName == "textured")
            marker.mesh_use_embedded_materials = true;
        typedef boost::filesystem::path fs_path;
        fs_path mesh_resource = fs_path(pbd_object.meshResourcePath).replace_extension(".dae");
        marker.mesh_resource = mesh_resource.string();
        marker.scale.x =  marker.scale.y = marker.scale.z = 0.001;
    }
    marker.color = pbd_object.color;
    marker.action = visualization_msgs::Marker::ADD;

    // Add first the marker of the detected pose
    marker.ns = pbd_object.providedBy + "_detected";

    if(!pbd_object.sampledPoses.size()){
        std::cerr << "Got a AsrObject without poses." << std::endl;
        std::exit(1);
    }

    marker.pose = pbd_object.sampledPoses.front().pose;
    addMarker(marker);

    // Add now the other poses
    marker.ns = pbd_object.providedBy + "_sampled";
    if (visualizeSampledPoses) {
        bool isNotFirst = false;
        for (const geometry_msgs::PoseWithCovariance &current_pose : pbd_object.sampledPoses)
        {
            if (isNotFirst)
            {
                marker.pose = current_pose.pose;
                addMarker(marker);
            }
            else
            {
                isNotFirst = true;
            }
        }
    }

    //Visualize Orientation-Axis(X-Axis)
    visualization_msgs::Marker axisMarker;
    axisMarker.header = pbd_object.header;

    axisMarker.type = visualization_msgs::Marker::ARROW;
    axisMarker.scale.y = axisMarker.scale.z = 0.01;
    axisMarker.scale.x = 0.1;
    Eigen::Quaterniond orientation;
    tf::quaternionMsgToEigen(pbd_object.sampledPoses.front().pose.orientation, orientation);
    Eigen::Quaterniond transformed_X_Axis = Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) * orientation;
    tf::quaternionEigenToMsg(transformed_X_Axis, axisMarker.pose.orientation);
    axisMarker.pose.position = pbd_object.sampledPoses.front().pose.position;
    axisMarker.color.a = 1.0;
    axisMarker.color.r = 0.0;
    axisMarker.color.g = 0.0;
    axisMarker.color.b = 1.0;
    axisMarker.action = visualization_msgs::Marker::ADD;
    axisMarker.ns = "ObjectOrientation_XAxis";
    addMarker(axisMarker);
}

}
