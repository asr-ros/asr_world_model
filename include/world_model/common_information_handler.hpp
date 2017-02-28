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


// ROS Main Inclusion
#include <ros/ros.h>


// ROS-wide Inclusion
#include <asr_msgs/AsrTypeAndId.h>
#include <asr_msgs/AsrObject.h>


// Local Inclusion
#include "asr_world_model/GetRecognizerName.h"
#include "asr_world_model/GetIntermediateObjectWeight.h"
#include "asr_world_model/GetAllObjectsList.h"
#include "asr_world_model/GetMissingObjectList.h"

#include "world_model/model/settings.hpp"
#include "world_model/model/model_type.hpp"
#include "world_model/model/model_object.hpp"
#include "world_model/helper/debug_helper.hpp"

namespace world_model
{

class CommonInformationHandler
{
public:
    /* ----------------- Public members  ------------------  */
    // Wrapped Constants

    // GetMissingObjectList
    static const inline std::string GetGetMissingObjectListServiceName ()
    { return "get_missing_object_list"; }

    // GetRecognizerName
    static const inline std::string GetGetRecognizerNameServiceName()
    { return "get_recognizer_name"; }

    // GetAllObjectsList
    static const inline std::string GetGetAllObjectsListServiceName()
    { return "get_all_objects_list"; }

    // GetIntermediateObjectWeight
    static const inline std::string GetGetIntermediateObjectWeightServiceName()
    { return "get_intermediate_object_weight"; }

private:
    /* ----------------- Private members  ------------------  */

    DebugHelperPtr debug_helper_ptr_;

    ModelTypePtrPerTypeMapPtr model_type_ptr_per_type_map_ptr_;
    SettingsPtr settings_ptr_;

public:
    /* ----------------- Public functions  ------------------  */
    /*!
     * \brief Creates a new instance of CommonInformationHandler.
     */
    CommonInformationHandler(ModelTypePtrPerTypeMapPtr model_type_ptr_per_type_map_ptr, SettingsPtr settings_ptr);

    /*!
     * \brief get the associated recognizer name for a given object
     * \param request the associated request object of the service call
     * \param response the associated response object of the service
     * \return the success of the service call.
     */
    bool processGetRecognizerNameServiceCall(asr_world_model::GetRecognizerName::Request &request,
                                             asr_world_model::GetRecognizerName::Response &response);

    /*!
     * \brief get the associated Intermediate Object Weight for a given object
     * \param request the associated request object of the service call
     * \param response the associated response object of the service
     * \return the success of the service call.
     */
    bool processGetIntermediateObjectWeightServiceCall(asr_world_model::GetIntermediateObjectWeight::Request &request,
                                                       asr_world_model::GetIntermediateObjectWeight::Response &response);

    /*!
     * \brief returns all objects in the current scenes
     * \param request the associated request object of the service call
     * \param response the associated response object of the service
     * \return the success of the service call.
     */
    bool processGetAllObjectsListServiceCall(asr_world_model::GetAllObjectsList::Request &request,
                                             asr_world_model::GetAllObjectsList::Response &response);

    /*!
     * \brief returns all missing objects in the current scene
     * \param request the associated request object of the service call
     * \param response the associated response object of the service
     * \return the success of the service call.
     */
    bool processGetMissingObjectListServiceCall(asr_world_model::GetMissingObjectList::Request &request,
                                                asr_world_model::GetMissingObjectList::Response &response);

private:
    void pushTypeAndId(std::vector<asr_msgs::AsrTypeAndId> &typeAndIds, const std::string type, const std::string identifier) const;
};

}

