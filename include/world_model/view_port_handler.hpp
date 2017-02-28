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


// ROS Main Inclusion
#include <ros/ros.h>


// ROS-wide Inclusion
#include <asr_msgs/AsrViewport.h>


// Local Inclusion
#include "asr_world_model/EmptyViewportList.h"
#include "asr_world_model/GetViewportList.h"
#include "asr_world_model/PushViewport.h"
#include "asr_world_model/FilterViewportDependingOnAlreadyVisitedViewports.h"

#include "world_model/model/settings.hpp"

#include "world_model/helper/debug_helper.hpp"
#include "world_model/helper/pose_helper.hpp"

namespace world_model
{

typedef std::vector<asr_msgs::AsrViewport> ViewportList;
typedef boost::shared_ptr<ViewportList> ViewportListPtr;

std::ostream& operator<<(std::ostream &strm, const ViewportList &viewport_list);
std::ostream& operator<<(std::ostream &strm, const ViewportListPtr &viewport_list_ptr);

/*!
 * \brief WorldModel class provides services for adding the viewports of the next best views to a list and retrieve them. Additionally, it provides services for adding objects, detected by object localization and retrieve them.
 */
class ViewPortHandler
{
public:
    /* ----------------- Public members  ------------------  */
    // Wrapped Constants

    // PushViewport
    static const inline std::string GetPushViewportServiceName ()
    { return "push_viewport"; }

    // EmptyViewportList
    static const inline std::string GetEmptyViewportListServiceName()
    { return "empty_viewport_list"; }

    // GetViewportList
    static const inline std::string GetGetViewportListServiceName()
    { return "get_viewport_list"; }

    // FilterViewportDependingOnAlreadyVisitedViewports
    static const inline std::string GetFilterViewportDependingOnAlreadyVisitedViewportsName()
    { return "filter_viewport_depending_on_already_visited_viewports"; }

private:
    /* ----------------- Private members  ------------------  */

    // Vars
    std::size_t number_of_all_viewports;
    ViewportListPtr viewport_list_ptr_;

    DebugHelperPtr debug_helper_ptr_;
    PoseHelperPtr pose_helper_ptr_;
    SettingsPtr settings_ptr_;

    void filterObjectTypesOfViewport(asr_msgs::AsrViewport &viewport_to_filter, const asr_msgs::AsrViewport &filter_viewport);

public:
    /* ----------------- Public functions  ------------------  */
    /*!
     * \brief Creates a new instance of the ViewPortHandler
     */
    ViewPortHandler(SettingsPtr settings_ptr);

    /*!
     * \brief Removes the whole next best view viewports from list if request.object_type is
     * set to "all" in the other case just the viewports associated with the given object type.
     * \param request the associated request object of the service call
     * \param response the associated response object of the service
     * \return the success of the service call.
     */
    bool processEmptyViewportListServiceCall(asr_world_model::EmptyViewportList::Request &request,
                                             asr_world_model::EmptyViewportList::Response &response);

    /*!
     * \brief Returns the whole list of next best view viewports if request.object_type is
     * set to "all" else just the subset which matches the object type given in request.object_type.
     * \param request the associated request object of the service call
     * \param response the associated response object of the service
     * \return the success of the service call.
     */
    bool processGetViewportListServiceCall(asr_world_model::GetViewportList::Request &request,
                                           asr_world_model::GetViewportList::Response &response);

    /*!
     * \brief Pushes a next best view viewport to a list.
     * \param request the associated request object of the service call
     * \param response the associated response object of the service call
     * \return the success of the service call.
     */
    bool processPushViewportServiceCall(asr_world_model::PushViewport::Request &request,
                                        asr_world_model::PushViewport::Response &response);

    /*!
     * \brief Filter the objects of the viewport depending on already visited viewports.
     * \param request the associated request object of the service call
     * \param response the associated response object of the service call
     * \return the success of the service call.
     */
    bool processFilterViewportDependingOnAlreadyVisitedViewportsVisited(asr_world_model::FilterViewportDependingOnAlreadyVisitedViewports::Request &request,
                                        asr_world_model::FilterViewportDependingOnAlreadyVisitedViewports::Response &response);

};

}
