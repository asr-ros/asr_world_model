/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Hutmacher Robin, Karrenbauer Oliver, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Trautmann Jeremias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "world_model/view_port_handler.hpp"

namespace world_model
{

ViewPortHandler::ViewPortHandler(SettingsPtr settings_ptr) : number_of_all_viewports(0), viewport_list_ptr_(new ViewportList()), settings_ptr_(settings_ptr)
{
    debug_helper_ptr_ = DebugHelper::getInstance();
    pose_helper_ptr_ = PoseHelper::getInstance();
    ROS_INFO_STREAM("ViewPortHandler initialized");
}

bool ViewPortHandler::processPushViewportServiceCall(asr_world_model::PushViewport::Request &request,
                                                     asr_world_model::PushViewport::Response &response)
{
    debug_helper_ptr_->write(std::stringstream() << "Calling processPushViewportServiceCall", (DebugHelper::SERVICE_CALLS + DebugHelper::VIEW_PORT));

    viewport_list_ptr_->push_back(request.viewport);
    ++number_of_all_viewports;

    debug_helper_ptr_->write(std::stringstream() << "Pushed viewport:\n" << request.viewport, DebugHelper::VIEW_PORT);

    return true;
}

bool ViewPortHandler::processEmptyViewportListServiceCall(asr_world_model::EmptyViewportList::Request &request,
                                                          asr_world_model::EmptyViewportList::Response &response)
{
    const std::string &object_type = request.object_type;
    debug_helper_ptr_->write(std::stringstream() << "Calling processEmptyViewportListServiceCall for object_type: " << object_type, (DebugHelper::SERVICE_CALLS + DebugHelper::VIEW_PORT));

    if (std::string("all") == object_type || std::string("") == object_type) {
        viewport_list_ptr_->clear();
        number_of_all_viewports = 0;
    } else {
        ViewportList::iterator viewportListIt = viewport_list_ptr_->begin();
        while (viewportListIt != viewport_list_ptr_->end()) {
            std::vector<std::string>::iterator foundObjectIt = std::find(viewportListIt->object_type_name_list.begin(), viewportListIt->object_type_name_list.end(), object_type);
            if (foundObjectIt != viewportListIt->object_type_name_list.end()) {
                // When it is not the last object in the viewport erase only the type
                if (viewportListIt->object_type_name_list.size() == 1) {
                    viewportListIt = viewport_list_ptr_->erase(viewportListIt);
                    continue;
                } else {
                    viewportListIt->object_type_name_list.erase(foundObjectIt);
                }
            }
            ++viewportListIt;
        }
    }

    return true;
}

bool ViewPortHandler::processGetViewportListServiceCall(asr_world_model::GetViewportList::Request &request,
                                                        asr_world_model::GetViewportList::Response &response)
{
    const std::string &object_type = request.object_type;
    debug_helper_ptr_->write(std::stringstream() << "Calling processGetViewportListServiceCall for object_type: " << object_type, (DebugHelper::SERVICE_CALLS + DebugHelper::VIEW_PORT));

    if (std::string("all") == object_type || std::string("") == object_type) {
        for (const asr_msgs::AsrViewport &viewport : *viewport_list_ptr_) {
            response.viewport_list.push_back(viewport);
        }
    } else {
        for (asr_msgs::AsrViewport &viewport : *viewport_list_ptr_) {
            std::vector<std::string>::iterator foundObjectIt = std::find(viewport.object_type_name_list.begin(), viewport.object_type_name_list.end(), object_type);
            if (foundObjectIt != viewport.object_type_name_list.end()) {
                response.viewport_list.push_back(viewport);
            }
        }
    }

    debug_helper_ptr_->write(std::stringstream() << "Returning viewportList:\n" << response.viewport_list, DebugHelper::VIEW_PORT);
    return true;
}

bool ViewPortHandler::processFilterViewportDependingOnAlreadyVisitedViewportsVisited(asr_world_model::FilterViewportDependingOnAlreadyVisitedViewports::Request &request,
                                                                                     asr_world_model::FilterViewportDependingOnAlreadyVisitedViewports::Response &response)
{
    debug_helper_ptr_->write(std::stringstream() << "Calling processFilterViewportDependingOnAlreadyVisitedViewportsVisited", (DebugHelper::SERVICE_CALLS + DebugHelper::VIEW_PORT));

    asr_msgs::AsrViewport filteredViewport = request.viewport;
    response.isBeenFiltered = false;

    for (const asr_msgs::AsrViewport &alreadyVisitedViewport : *viewport_list_ptr_) {
        if (pose_helper_ptr_->checkPosesAreApproxEquale(alreadyVisitedViewport.pose, filteredViewport.pose,
                                                        settings_ptr_->viewport_position_distance_threshold, settings_ptr_->viewport_orientation_rad_distance_threshold)) {
            debug_helper_ptr_->write(std::stringstream() << "pose is approx equale to:\n" << alreadyVisitedViewport, DebugHelper::VIEW_PORT);
            response.isBeenFiltered = true;
            filterObjectTypesOfViewport(filteredViewport, alreadyVisitedViewport);
        }
    }

    response.filteredViewport = filteredViewport;
    debug_helper_ptr_->write(std::stringstream() << "response:\n" << response, DebugHelper::VIEW_PORT);
    return true;
}

void ViewPortHandler::filterObjectTypesOfViewport(asr_msgs::AsrViewport &viewport_to_filter, const asr_msgs::AsrViewport &filter_viewport) {
    std::vector<std::string> &objectTypesToFilter = viewport_to_filter.object_type_name_list;

    for (const std::string &filter_object_type : filter_viewport.object_type_name_list) {
        bool isFound = false;
        do {
            std::vector<std::string>::iterator iter = std::find(objectTypesToFilter.begin(), objectTypesToFilter.end(), filter_object_type);
            if (iter != objectTypesToFilter.end()) {
                objectTypesToFilter.erase(iter);
                isFound = true;
            }  else {
                isFound = false;
            }
        } while (isFound);
    }
}

std::ostream& operator<<(std::ostream &strm, const ViewportList &viewport_list) {
    for (const asr_msgs::AsrViewport &viewport : viewport_list) {
        strm << viewport;
    }
    return strm;
}

std::ostream& operator<<(std::ostream &strm, const ViewportListPtr &viewport_list_ptr) {
    return strm << *viewport_list_ptr;
}

}
