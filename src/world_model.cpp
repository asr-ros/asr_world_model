/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Hutmacher Robin, Karrenbauer Oliver, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Trautmann Jeremias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "world_model/world_model.hpp"

#include "asr_object_database/RecognizerList.h"
#include <boost/algorithm/string.hpp>

namespace world_model
{

WorldModel::WorldModel() :
    local_handle_(ros::this_node::getName()),
    mDynamicReconfigServer(local_handle_),
    model_type_ptr_per_type_map_ptr_(new ModelTypePtrPerTypeMap()),
    settings_ptr_(new Settings())
{
    debug_helper_ptr_ = DebugHelper::getInstance();

    dynamic_reconfigure::Server<asr_world_model::DynamicParametersConfig>::CallbackType f = boost::bind(&WorldModel::dynamicReconfigureCallback, this, _1, _2);
    mDynamicReconfigServer.setCallback(f);

    boost::shared_ptr<ros::Publisher> marker_publisher_ptr = boost::make_shared<ros::Publisher>(
                local_handle_.advertise<visualization_msgs::MarkerArray>(WorldModel::GetObjectVisualizationParamName(),
                                                                         10,
                                                                         true));

    boost::shared_ptr<ros::ServiceClient> object_type_client_ptr =
            boost::make_shared<ros::ServiceClient>(local_handle_.serviceClient<asr_object_database::ObjectMetaData>(WorldModel::GetClientGetObjectMetaDataServiceName()));

    found_object_handler_ptr_ = boost::shared_ptr<FoundObjectHandler>(new FoundObjectHandler(marker_publisher_ptr,
                                                                                             object_type_client_ptr,
                                                                                             model_type_ptr_per_type_map_ptr_,
                                                                                             settings_ptr_));
    view_port_handler_ptr_ = boost::shared_ptr<ViewPortHandler>(new ViewPortHandler(settings_ptr_));
    common_information_handler_ptr_ = boost::shared_ptr<CommonInformationHandler>(new CommonInformationHandler(model_type_ptr_per_type_map_ptr_, settings_ptr_));
    complete_patterns_handler_ptr_ = boost::shared_ptr<CompletePatternsHandler>(new CompletePatternsHandler());


    advertiseServices();

    ROS_INFO_STREAM("Initialization done");
}

void WorldModel::dynamicReconfigureCallback(asr_world_model::DynamicParametersConfig &config, uint32_t level) {
    ROS_INFO_STREAM("Dynamic reconfigure called with level: " << level);

    local_handle_.setParam("enable_object_sampling", config.enable_object_sampling);
    local_handle_.setParam("calculate_deviations", config.calculate_deviations);
    local_handle_.setParam("deviation_number_of_samples_position", config.deviation_number_of_samples_position);
    local_handle_.setParam("deviation_number_of_samples_orientation", config.deviation_number_of_samples_orientation);
    local_handle_.setParam("objects_to_sample", config.objects_to_sample);
    local_handle_.setParam("object_position_distance_threshold", config.object_position_distance_threshold);
    local_handle_.setParam("object_orientation_rad_distance_threshold", config.object_orientation_rad_distance_threshold);
    local_handle_.setParam("viewport_position_distance_threshold", config.viewport_position_distance_threshold);
    local_handle_.setParam("viewport_orientation_rad_distance_threshold", config.viewport_orientation_rad_distance_threshold);
    local_handle_.setParam("object_rating_min_count", config.object_rating_min_count);
    local_handle_.setParam("use_default_intermediate_object_weight", config.use_default_intermediate_object_weight);
    local_handle_.setParam("default_intermediate_object_weight", config.default_intermediate_object_weight);
    local_handle_.setParam("intermediate_object_weight_file_name", config.intermediate_object_weight_file_name);
    local_handle_.setParam("use_world_description", config.use_world_description);

    local_handle_.setParam("recognizers_string_map", config.recognizers_string_map);
    local_handle_.setParam("weight_string_map", config.weight_string_map);
    local_handle_.setParam("object_string_map", config.object_string_map);

    local_handle_.setParam("debugLevels", config.debugLevels);

    checkParametersFromOtherNode();
    initParams();
}

void WorldModel::checkParametersFromOtherNode() {
    if (local_handle_.hasParam("/rp_ism_node/bin_size")) {
        double oldBinSize = settings_ptr_->bin_size;
        local_handle_.getParam("/rp_ism_node/bin_size", settings_ptr_->bin_size);
        local_handle_.setParam("bin_size", settings_ptr_->bin_size);
        if (oldBinSize != settings_ptr_->bin_size) {
            debug_helper_ptr_->write(std::stringstream() << "/rp_ism_node/bin_size was updated to: " << settings_ptr_->bin_size, DebugHelper::PARAMETERS);
        }
    }
    if (local_handle_.hasParam("/rp_ism_node/maxProjectionAngleDeviation")) {
        double oldMaxProjectionAngleDeviation = settings_ptr_->maxProjectionAngleDeviation;
        local_handle_.getParam("/rp_ism_node/maxProjectionAngleDeviation", settings_ptr_->maxProjectionAngleDeviation);
        local_handle_.setParam("maxProjectionAngleDeviation", settings_ptr_->maxProjectionAngleDeviation);
        if (oldMaxProjectionAngleDeviation != settings_ptr_->maxProjectionAngleDeviation) {
            debug_helper_ptr_->write(std::stringstream() << "/rp_ism_node/maxProjectionAngleDeviation was updated to:" << settings_ptr_->maxProjectionAngleDeviation, DebugHelper::PARAMETERS);
        }
    }
    if (local_handle_.hasParam("/rp_ism_node/dbfilename")) {
        std::string oldDbfilename = settings_ptr_->dbfilename;
        local_handle_.getParam("/rp_ism_node/dbfilename", settings_ptr_->dbfilename);
        local_handle_.setParam("dbfilename", settings_ptr_->dbfilename);
        if (oldDbfilename != settings_ptr_->dbfilename) {
            debug_helper_ptr_->write(std::stringstream() << "/rp_ism_node/dbfilename was updated to: " << settings_ptr_->dbfilename, DebugHelper::PARAMETERS);
        }
    }
}

void WorldModel::initParams() {
    DebugHelper::resetInstance();
    PoseHelper::resetInstance();

    local_handle_.getParam("enable_object_sampling", settings_ptr_->enable_object_sampling);
    local_handle_.getParam("calculate_deviations", settings_ptr_->calculate_deviations);
    local_handle_.getParam("deviation_number_of_samples_position", settings_ptr_->deviation_number_of_samples_position);
    local_handle_.getParam("deviation_number_of_samples_orientation", settings_ptr_->deviation_number_of_samples_orientation);
    local_handle_.getParam("object_position_distance_threshold", settings_ptr_->object_position_distance_threshold);
    local_handle_.getParam("object_orientation_rad_distance_threshold", settings_ptr_->object_orientation_rad_distance_threshold);
    local_handle_.getParam("viewport_position_distance_threshold", settings_ptr_->viewport_position_distance_threshold);
    local_handle_.getParam("viewport_orientation_rad_distance_threshold", settings_ptr_->viewport_orientation_rad_distance_threshold);
    local_handle_.getParam("object_rating_min_count", settings_ptr_->object_rating_min_count);
    local_handle_.getParam("use_default_intermediate_object_weight", settings_ptr_->use_default_intermediate_object_weight);
    local_handle_.getParam("default_intermediate_object_weight", settings_ptr_->default_intermediate_object_weight);
    local_handle_.getParam("use_world_description", settings_ptr_->use_world_description);

    local_handle_.getParam("dbfilename", settings_ptr_->dbfilename);
    local_handle_.getParam("bin_size", settings_ptr_->bin_size);
    local_handle_.getParam("maxProjectionAngleDeviation", settings_ptr_->maxProjectionAngleDeviation);

    std::string objects_to_sample;
    local_handle_.getParam("objects_to_sample", objects_to_sample);
    settings_ptr_->objects_to_sample = *processStringVector<std::string>(objects_to_sample);

    debug_helper_ptr_->write(std::stringstream() << '\n' << settings_ptr_, DebugHelper::PARAMETERS);

    if (settings_ptr_->use_world_description) {
        initMapsFromWorldDescription();
    } else {
        initMapsFromSqlTable();
    }

    //Log parsed values
    debug_helper_ptr_->write(std::stringstream() << "Parsed ModelObjects:" << '\n' << model_type_ptr_per_type_map_ptr_, DebugHelper::PARAMETERS);

    ROS_INFO_STREAM("Parameters initialized");
}

void WorldModel::initMapsFromWorldDescription() {
    std::string recognizers_string_map_;
    local_handle_.getParam("recognizers_string_map", recognizers_string_map_);
    boost::shared_ptr<std::map<std::string, std::string>> recognizers_map_ptr_ = processStringMap<std::string>(recognizers_string_map_);

    for (const std::pair<std::string, std::string> &recongnizer_pair  : *recognizers_map_ptr_) {
        if (model_type_ptr_per_type_map_ptr_->find(recongnizer_pair.first) == model_type_ptr_per_type_map_ptr_->end()) {
            model_type_ptr_per_type_map_ptr_->insert(std::make_pair(recongnizer_pair.first, ModelTypePtr(new ModelType(recongnizer_pair.first))));
        }
        (*model_type_ptr_per_type_map_ptr_)[recongnizer_pair.first]->recognizerName = recongnizer_pair.second;
    }


    std::string weight_string_map_;
    local_handle_.getParam("weight_string_map", weight_string_map_);
    boost::shared_ptr<std::map<std::string, double>> weight_map_ptr = processStringMap<double>(weight_string_map_);

    for (const std::pair<std::string, double> &weight_pair  : *weight_map_ptr) {
        if (model_type_ptr_per_type_map_ptr_->find(weight_pair.first) == model_type_ptr_per_type_map_ptr_->end()) {
            model_type_ptr_per_type_map_ptr_->insert(std::make_pair(weight_pair.first, ModelTypePtr(new ModelType(weight_pair.first))));
        }
        (*model_type_ptr_per_type_map_ptr_)[weight_pair.first]->weight = weight_pair.second;
    }


    std::string object_string_map;
    local_handle_.getParam("object_string_map", object_string_map);
    boost::shared_ptr<std::map<std::string, int>> parsedObjectMapPtr = processStringMap<int>(object_string_map);

    for (const std::pair<std::string, int> &parsedObjectMap : *parsedObjectMapPtr) {
        const std::string &typeAndId = parsedObjectMap.first;
        std::size_t found = typeAndId.find("&");
        if (found != std::string::npos) {

            const std::string &type = typeAndId.substr(0,found);
            const std::string &observed_id = typeAndId.substr(found+1);
            if (model_type_ptr_per_type_map_ptr_->find(type) == model_type_ptr_per_type_map_ptr_->end()) {
                model_type_ptr_per_type_map_ptr_->insert(std::make_pair(type, ModelTypePtr(new ModelType(type))));
            }

            if (model_type_ptr_per_type_map_ptr_->find(type)->second->model_object_ptr_per_id_map.find(observed_id) ==
                    model_type_ptr_per_type_map_ptr_->find(type)->second->model_object_ptr_per_id_map.end()) {
                model_type_ptr_per_type_map_ptr_->find(type)->second->model_object_ptr_per_id_map.insert(
                            std::make_pair(observed_id, ModelObjectPtr(new ModelObject(observed_id))));
            }
            (*model_type_ptr_per_type_map_ptr_)[type]->model_object_ptr_per_id_map[observed_id]->objectCount = parsedObjectMap.second;
        }
    }

}

void WorldModel::initMapsFromSqlTable() {
    ros::ServiceClient recognizerListServiceClient = local_handle_.serviceClient<asr_object_database::RecognizerList>(WorldModel::GetClientGetRecognizerListServiceName());
    //Duration is buggy here
    ROS_INFO("Wait for asr_object_database to existence");
    recognizerListServiceClient.waitForExistence();

    asr_object_database::RecognizerList recognizerList;
    if (!recognizerListServiceClient.call(recognizerList)) {
        ROS_ERROR("Get RecognizerList service call to asr_object_database was not successful");
        ros::shutdown();
        return;
    }

    ISM::TableHelperPtr table_helper = ISM::TableHelperPtr(new ISM::TableHelper(settings_ptr_->dbfilename));

    std::set<std::pair<std::string, std::string>> objectTypesAndIds = table_helper->getObjectTypesAndIdsFromModelObjects();
    for (const std::pair<std::string, std::string> &objectTypesAndIdFromRecordedObject : table_helper->getObjectTypesAndIdsFromRecordedObjects()) {
        objectTypesAndIds.insert(objectTypesAndIdFromRecordedObject);
    }
    const std::map<std::string, boost::filesystem::path> &ressourcePaths = table_helper->getRessourcePaths();
    std::map<std::string, std::map<std::string, unsigned int>> temp_object_count_per_type_and_id_map;

    for (const std::pair<std::string, std::string> &objectTypeAndId : objectTypesAndIds) {
        if (objectTypeAndId.first.find("_sub") != std::string::npos) {
            // skip scenes
            continue;
        }
        std::map<std::string, boost::filesystem::path>::const_iterator path_it = ressourcePaths.find(objectTypeAndId.first);
        if (path_it == ressourcePaths.end()) {
            ROS_WARN_STREAM("No resourcePath for type/id: " << objectTypeAndId.first << "/" << objectTypeAndId.second <<
                            " in ISM_Table found -> will not be used in world_model");
            continue;
        }

        if (model_type_ptr_per_type_map_ptr_->find(objectTypeAndId.first) == model_type_ptr_per_type_map_ptr_->end()) {
            model_type_ptr_per_type_map_ptr_->insert(std::make_pair(objectTypeAndId.first, ModelTypePtr(new ModelType(objectTypeAndId.first))));
        }

        if (model_type_ptr_per_type_map_ptr_->find(objectTypeAndId.first)->second->model_object_ptr_per_id_map.find(objectTypeAndId.second) ==
                model_type_ptr_per_type_map_ptr_->find(objectTypeAndId.first)->second->model_object_ptr_per_id_map.end()) {
            model_type_ptr_per_type_map_ptr_->find(objectTypeAndId.first)->second->model_object_ptr_per_id_map.insert(
                        std::make_pair(objectTypeAndId.second, ModelObjectPtr(new ModelObject(objectTypeAndId.second))));
        }

        for (const std::string &recognizer_name : recognizerList.response.recognizer_list) {
            if (path_it->second.string().find(recognizer_name) != std::string::npos) {
                //The recognizer_name has to be in the path to the asr_object_database to know with which recognizer the object was learned
                (*model_type_ptr_per_type_map_ptr_)[objectTypeAndId.first]->recognizerName = recognizer_name;
            }
        }


        //temp_object_count_per_type_and_id_map will be used later on to determine the number of objects of same type and id
        if (temp_object_count_per_type_and_id_map.find(objectTypeAndId.first) == temp_object_count_per_type_and_id_map.end()) {
            temp_object_count_per_type_and_id_map.insert(std::make_pair(objectTypeAndId.first, std::map<std::string, unsigned int>()));
        }
        temp_object_count_per_type_and_id_map.find(objectTypeAndId.first)->second.insert(std::make_pair(objectTypeAndId.second, 0));
    }


    //Iterate over all recorded sets to gain the number of trained objects per type and id
    const std::vector<int> &setIds = table_helper->getSetIds();
    for (const int &setId : setIds) {
        const ISM::ObjectSetPtr &objectSetPtr = table_helper->getRecordedObjectSet(setId);
        for (const ISM::ObjectPtr &object : objectSetPtr->objects) {
            ++temp_object_count_per_type_and_id_map.find(object->type)->second.find(object->observedId)->second;
        }

        for (const std::pair<std::string, ModelTypePtr> &model_type_ptr_per_type_pair: *model_type_ptr_per_type_map_ptr_) {
            for (const std::pair<std::string, ModelObjectPtr> &model_object_ptr_per_id_pair : model_type_ptr_per_type_pair.second->model_object_ptr_per_id_map) {
                unsigned int &oldMaxValue = model_object_ptr_per_id_pair.second->objectCount;
                unsigned int &currentValue = temp_object_count_per_type_and_id_map.find(model_type_ptr_per_type_pair.first)->second.find(model_object_ptr_per_id_pair.first)->second;
                //Take the max count of all iterated sets
                model_object_ptr_per_id_pair.second->objectCount = std::max(oldMaxValue, currentValue);
                //Reset of temp value
                temp_object_count_per_type_and_id_map.find(model_type_ptr_per_type_pair.first)->second.find(model_object_ptr_per_id_pair.first)->second = 0;
            }
        }

    }

    initIntermediateObjectWeights(table_helper);
}

void WorldModel::initIntermediateObjectWeights(const ISM::TableHelperPtr &table_helper) {
    if (settings_ptr_->use_default_intermediate_object_weight) {
        for (const std::pair<std::string, ModelTypePtr> &model_type_ptr_per_type_pair: *model_type_ptr_per_type_map_ptr_) {
            model_type_ptr_per_type_pair.second->weight = settings_ptr_->default_intermediate_object_weight;
        }
        return;
    }


    local_handle_.getParam("intermediate_object_weight_file_name", settings_ptr_->intermediate_object_weight_file_name);

    std::vector<std::string> pathSplit;
    boost::split(pathSplit, settings_ptr_->dbfilename, boost::is_any_of("/"));
    std::string dbName = pathSplit[pathSplit.size()-1];

    const std::string placeholder = "XXX";
    std::string::size_type pos;
    if ((pos = settings_ptr_->intermediate_object_weight_file_name.find(placeholder)) != std::string::npos) {
        settings_ptr_->intermediate_object_weight_file_name.replace(pos, placeholder.length(), dbName);
    }

    debug_helper_ptr_->write(std::stringstream() << "intermediate_object_weight_file_name: " << settings_ptr_->intermediate_object_weight_file_name, DebugHelper::PARAMETERS);

    if (!std::ifstream(settings_ptr_->intermediate_object_weight_file_name.c_str()).good()) {
        ROS_INFO_STREAM("intermediate_object_weight_file: " << settings_ptr_->intermediate_object_weight_file_name << " not found. Execute itermediate_object_generator to generate file");
        system("roslaunch asr_intermediate_object_generator asr_intermediate_object_generator.launch");
    }
    std::ifstream intermediate_object_weight_file(settings_ptr_->intermediate_object_weight_file_name.c_str(), std::ifstream::in);
    // buffer and doc have to be declared here in the outer scope
    std::vector<char> buffer;
    rapidxml::xml_document<> doc;
    rapidxml::xml_node<> *interObjNode;
    if (intermediate_object_weight_file) {
        buffer = std::vector<char>((std::istreambuf_iterator<char>(intermediate_object_weight_file)), std::istreambuf_iterator<char>());
        buffer.push_back('\0');
        // Parse the buffer using the xml file parsing library into doc
        doc.parse<0>(&buffer[0]);
        interObjNode = doc.first_node("InterObj");
    } else {
        ROS_INFO_STREAM("intermediate_object_weight_file still not found (there must be a problem with the itermediate_object_generator)."
                        << " Check if weights are in sqlTable or use default_intermediate_object_weight");
    }


    std::map<std::string, std::map<std::string, std::string>> modelWeightsPerTypeAndId = table_helper->getModelWeightsPerTypeAndId();

    for (const std::pair<std::string, ModelTypePtr> &model_type_ptr_per_type_pair: *model_type_ptr_per_type_map_ptr_) {
        for (const std::pair<std::string, ModelObjectPtr> &model_object_ptr_per_id_pair : model_type_ptr_per_type_pair.second->model_object_ptr_per_id_map) {

            double weight = 0.0;
            bool weightFound = false;
            if (interObjNode) {
                for (rapidxml::xml_node<> *obj_node = interObjNode->first_node("obj"); obj_node; obj_node = obj_node->next_sibling()) {
                    std::string name = boost::lexical_cast<std::string>(obj_node->first_attribute("name")->value());
                    std::string id = boost::lexical_cast<std::string>(obj_node->first_attribute("observedId")->value());
                    if (model_type_ptr_per_type_pair.first == name && model_object_ptr_per_id_pair.first == id) {
                        weight = std::max(weight, boost::lexical_cast<double>(obj_node->value()));
                        weightFound = true;
                    }
                }
            }
            if (interObjNode && !weightFound) {
                ROS_INFO_STREAM("No intermediate_object_weight for type/id: " <<  model_type_ptr_per_type_pair.first << "/" << model_object_ptr_per_id_pair.first << " in xml_file found."
                                << " Check if weights are in sqlTable or use default_intermediate_object_weight");
            }
            if (!weightFound) {
                for (const std::pair<std::string, std::map<std::string, std::string>> &modelWeightsPerTypeAndIdPair: modelWeightsPerTypeAndId) {
                    for (const std::pair<std::string, std::string> &modelWeightsPerIdPair : modelWeightsPerTypeAndIdPair.second) {
                        if (model_type_ptr_per_type_pair.first == modelWeightsPerTypeAndIdPair.first && model_object_ptr_per_id_pair.first == modelWeightsPerIdPair.first) {
                            weight = std::max(weight, boost::lexical_cast<double>(modelWeightsPerIdPair.second));
                            weightFound = true;
                        }
                    }
                }
            }
            if (!weightFound) {
                ROS_INFO_STREAM("No intermediate_object_weight for type/id: " <<  model_type_ptr_per_type_pair.first << "/" << model_object_ptr_per_id_pair.first << " in sqlTable found. Use default_intermediate_object_weight");
                weight = settings_ptr_->default_intermediate_object_weight;
            }
            //need max, because weights will be calculated per type AND Id, but we save only per type
            model_type_ptr_per_type_pair.second->weight = std::max(weight,  model_type_ptr_per_type_pair.second->weight);
        }
    }
}

void WorldModel::advertiseServices() {
    //ViewPortHandler services
    push_viewport_service_server_ = local_handle_.advertiseService(ViewPortHandler::GetPushViewportServiceName(),
                                                                   &ViewPortHandler::processPushViewportServiceCall,
                                                                   view_port_handler_ptr_);

    empty_viewport_list_service_server_ = local_handle_.advertiseService(ViewPortHandler::GetEmptyViewportListServiceName(),
                                                                         &ViewPortHandler::processEmptyViewportListServiceCall,
                                                                         view_port_handler_ptr_);

    get_viewport_list_service_server_ = local_handle_.advertiseService(ViewPortHandler::GetGetViewportListServiceName(),
                                                                       &ViewPortHandler::processGetViewportListServiceCall,
                                                                       view_port_handler_ptr_);

    filter_viewport_depending_on_already_visited_viewports = local_handle_.advertiseService(ViewPortHandler::GetFilterViewportDependingOnAlreadyVisitedViewportsName(),
                                                                         &ViewPortHandler::processFilterViewportDependingOnAlreadyVisitedViewportsVisited,
                                                                         view_port_handler_ptr_);

    //FoundObjectHandler services
    push_found_object_service_server_ = local_handle_.advertiseService(FoundObjectHandler::GetPushFoundObjectServiceName(),
                                                                       &FoundObjectHandler::processPushFoundObjectServiceCall,
                                                                       found_object_handler_ptr_);

    push_found_object_list_service_server_ = local_handle_.advertiseService(FoundObjectHandler::GetPushFoundObjectListServiceName(),
                                                                            &FoundObjectHandler::processPushFoundObjectListServiceCall,
                                                                            found_object_handler_ptr_);

    // so that we can call checkParametersFromOtherNode
    empty_found_object_list_service_server_ = local_handle_.advertiseService(FoundObjectHandler::GetEmptyFoundObjectListServiceName(),
                                                                             &WorldModel::processEmptyFoundObjectListServiceCall,
                                                                             this);

    get_found_object_list_service_server_ = local_handle_.advertiseService(FoundObjectHandler::GetGetFoundObjectListServiceName(),
                                                                           &FoundObjectHandler::processGetFoundObjectListServiceCall,
                                                                           found_object_handler_ptr_);

    visualize_sampled_poses_service_server_ = local_handle_.advertiseService(FoundObjectHandler::GetVisualizeSampledPosesName(),
                                                                             &FoundObjectHandler::processVisualizeSampledPosesCall,
                                                                             found_object_handler_ptr_);

    //CommonInformationHandler services
    get_recognizer_name_service_server_ = local_handle_.advertiseService(CommonInformationHandler::GetGetRecognizerNameServiceName(),
                                                                         &CommonInformationHandler::processGetRecognizerNameServiceCall,
                                                                         common_information_handler_ptr_);

    get_intermediate_object_weight_service_server_ = local_handle_.advertiseService(CommonInformationHandler::GetGetIntermediateObjectWeightServiceName(),
                                                                                    &CommonInformationHandler::processGetIntermediateObjectWeightServiceCall,
                                                                                    common_information_handler_ptr_);

    get_all_objects_list_service_server_ = local_handle_.advertiseService(CommonInformationHandler::GetGetAllObjectsListServiceName(),
                                                                          &WorldModel::processGetAllObjectsListServiceCall,
                                                                          this);

    get_missing_object_list_service_server_ = local_handle_.advertiseService(CommonInformationHandler::GetGetMissingObjectListServiceName(),
                                                                             &CommonInformationHandler::processGetMissingObjectListServiceCall,
                                                                             common_information_handler_ptr_);

    //CompletePatternsHandler services
    get_complete_patterns_service_server_ = local_handle_.advertiseService(CompletePatternsHandler::GetGetCompletePatternsServiceName(),
                                                                           &CompletePatternsHandler::processGetCompletePatternsServiceCall,
                                                                           complete_patterns_handler_ptr_);

    empty_complete_patterns_service_server_ = local_handle_.advertiseService(CompletePatternsHandler::GetEmptyCompletePatternsServiceName(),
                                                                             &CompletePatternsHandler::processEmptyCompletePatternsServiceCall,
                                                                             complete_patterns_handler_ptr_);

    push_complete_patterns_service_server_ = local_handle_.advertiseService(CompletePatternsHandler::GetPushCompletePatternsServiceName(),
                                                                            &CompletePatternsHandler::processPushCompletePatternsCall,
                                                                            complete_patterns_handler_ptr_);
}

bool WorldModel::processEmptyFoundObjectListServiceCall(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
    bool success = found_object_handler_ptr_->processEmptyFoundObjectListServiceCall(request, response);

    std::string mydbfilename;
    local_handle_.getParam("dbfilename", mydbfilename);
    checkParametersFromOtherNode();
    std::string newMydbfilename;
    local_handle_.getParam("dbfilename", newMydbfilename);

    if (mydbfilename != newMydbfilename) {
        ROS_INFO_STREAM("dbfilename changed");
        model_type_ptr_per_type_map_ptr_->clear();
        initParams();
    }
    return success;
}

bool WorldModel::processGetAllObjectsListServiceCall(asr_world_model::GetAllObjectsList::Request &request, asr_world_model::GetAllObjectsList::Response &response) {
    std::string mydbfilename;
    local_handle_.getParam("dbfilename", mydbfilename);
    checkParametersFromOtherNode();
    std::string newMydbfilename;
    local_handle_.getParam("dbfilename", newMydbfilename);

    if (mydbfilename != newMydbfilename) {
        ROS_INFO_STREAM("dbfilename changed");
        model_type_ptr_per_type_map_ptr_->clear();
        initParams();
    }
    return common_information_handler_ptr_->processGetAllObjectsListServiceCall(request, response);
}

}
