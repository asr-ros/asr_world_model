/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Hutmacher Robin, Karrenbauer Oliver, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Trautmann Jeremias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "world_model/common_information_handler.hpp"

namespace world_model
{

CommonInformationHandler::CommonInformationHandler(ModelTypePtrPerTypeMapPtr model_type_ptr_per_type_map_ptr, SettingsPtr settings_ptr) :
    model_type_ptr_per_type_map_ptr_(model_type_ptr_per_type_map_ptr), settings_ptr_(settings_ptr)
{
    debug_helper_ptr_ = DebugHelper::getInstance();
    ROS_INFO_STREAM("CommonInformationHandler initialized");
}


bool CommonInformationHandler::processGetRecognizerNameServiceCall(asr_world_model::GetRecognizerName::Request &request,
                                                                   asr_world_model::GetRecognizerName::Response &response)
{
    debug_helper_ptr_->write(std::stringstream() << "Calling processGetRecognizerNameServiceCall for object_type: " << request.object_type, (DebugHelper::SERVICE_CALLS + DebugHelper::COMMON_INFORMATION));

    ModelTypePtrPerTypeMap::iterator perTypeit = model_type_ptr_per_type_map_ptr_->find(request.object_type);
    if (perTypeit == model_type_ptr_per_type_map_ptr_->end()) {
        ROS_WARN_STREAM("No recognizer name found for this object type: " << request.object_type);
        return false;
    }

    response.recognizer_name = perTypeit->second->recognizerName;
    debug_helper_ptr_->write(std::stringstream() << "Returning recognizer name: " << response.recognizer_name, DebugHelper::COMMON_INFORMATION);
    return true;
}


bool CommonInformationHandler::processGetIntermediateObjectWeightServiceCall(asr_world_model::GetIntermediateObjectWeight::Request &request,
                                                                             asr_world_model::GetIntermediateObjectWeight::Response &response)
{
    debug_helper_ptr_->write(std::stringstream() << "Calling processGetIntermediateObjectWeightServiceCall for object_type: " << request.object_type, (DebugHelper::SERVICE_CALLS + DebugHelper::COMMON_INFORMATION));

    ModelTypePtrPerTypeMap::iterator perTypeit = model_type_ptr_per_type_map_ptr_->find(request.object_type);
    if (perTypeit == model_type_ptr_per_type_map_ptr_->end()) {
        ROS_WARN_STREAM("No weight found for this object type: " << request.object_type);
        return false;
    }

    response.value = perTypeit->second->weight;
    debug_helper_ptr_->write(std::stringstream() << "Returning weight: " << response.value, DebugHelper::COMMON_INFORMATION);
    return true;
}

bool CommonInformationHandler::processGetAllObjectsListServiceCall(asr_world_model::GetAllObjectsList::Request &request,
                                                                   asr_world_model::GetAllObjectsList::Response &response) {
    debug_helper_ptr_->write(std::stringstream() << "Calling processGetAllObjectsListServiceCall", (DebugHelper::SERVICE_CALLS + DebugHelper::COMMON_INFORMATION));

    debug_helper_ptr_->write(std::stringstream() << "All objects: ", DebugHelper::COMMON_INFORMATION);

    for (const std::pair<std::string, ModelTypePtr> &model_type_ptr_per_type_pair: *model_type_ptr_per_type_map_ptr_) {
        for (const std::pair<std::string, ModelObjectPtr> &model_object_ptr_per_id_pair : model_type_ptr_per_type_pair.second->model_object_ptr_per_id_map) {
            pushTypeAndId(response.allObjects, model_type_ptr_per_type_pair.first, model_object_ptr_per_id_pair.first);
        }
    }
    response.scenePath = settings_ptr_->dbfilename;
    debug_helper_ptr_->write(std::stringstream() << response.allObjects.size() << " object(s) in scenes", DebugHelper::COMMON_INFORMATION);
    debug_helper_ptr_->write(std::stringstream() << "Current scenePath: " << response.scenePath, DebugHelper::COMMON_INFORMATION);
    return true;
}

bool CommonInformationHandler::processGetMissingObjectListServiceCall(asr_world_model::GetMissingObjectList::Request &request, asr_world_model::GetMissingObjectList::Response &response)
{
    debug_helper_ptr_->write(std::stringstream() << "Calling processGetMissingObjectListServiceCall", (DebugHelper::SERVICE_CALLS + DebugHelper::COMMON_INFORMATION));

    debug_helper_ptr_->write(std::stringstream() << "Missing objects: ", DebugHelper::COMMON_INFORMATION);

    for (const std::pair<std::string, ModelTypePtr> &model_type_ptr_per_type_pair: *model_type_ptr_per_type_map_ptr_) {
        for (const std::pair<std::string, ModelObjectPtr> &model_object_ptr_per_id_pair : model_type_ptr_per_type_pair.second->model_object_ptr_per_id_map) {
            if (model_object_ptr_per_id_pair.second->bestHypotheses.size() < model_object_ptr_per_id_pair.second->objectCount) {
                pushTypeAndId(response.missingObjects, model_type_ptr_per_type_pair.first, model_object_ptr_per_id_pair.first);
            }
        }
    }

    debug_helper_ptr_->write(std::stringstream() << response.missingObjects.size() << " object(s) still missing", DebugHelper::COMMON_INFORMATION);
    return true;
}

void CommonInformationHandler::pushTypeAndId(std::vector<asr_msgs::AsrTypeAndId> &typeAndIds, const std::string type, const std::string identifier) const {
    asr_msgs::AsrTypeAndId typeAndId;
    typeAndId.type = type;
    typeAndId.identifier = identifier;
    typeAndIds.push_back(typeAndId);
    debug_helper_ptr_->write(std::stringstream() << " - " << type << ", " << identifier, DebugHelper::COMMON_INFORMATION);
}

}
