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
#include <boost/lexical_cast.hpp>
#include <vector>
#include <map>
#include <list>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <memory>
#include <rapidxml.hpp>

// ROS Main Inclusion
#include <ros/ros.h>

// ROS-wide Inclusion
#include <ISM/utility/TableHelper.hpp>
#include <asr_msgs/AsrObject.h>
#include <dynamic_reconfigure/server.h>

#include <asr_world_model/DynamicParametersConfig.h>

#include "world_model/model/settings.hpp"
#include "world_model/model/model_type.hpp"
#include "world_model/model/model_object.hpp"
#include "world_model/helper/debug_helper.hpp"
#include "world_model/view_port_handler.hpp"
#include "world_model/found_object_handler.hpp"
#include "world_model/common_information_handler.hpp"
#include "world_model/complete_patterns_handler.hpp"


namespace world_model
{


/*!
 * \brief WorldModel class provides services for adding the viewports of the next best views to a list and retrieve them. Additionally, it provides services for adding objects, detected by object localization and retrieve them.
 */
class WorldModel
{
public:
    /* ----------------- Public members  ------------------  */
    // Wrapped Constants

    // Object visualization
    static const inline std::string GetObjectVisualizationParamName()
    { return "found_object_visualization"; }

private:
    static const inline std::string GetClientGetObjectMetaDataServiceName()
    { return "/asr_object_database/object_meta_data"; }

    static const inline std::string GetClientGetRecognizerListServiceName()
    { return "/asr_object_database/recognizer_list"; }
private:
    /* ----------------- Private members  ------------------  */
    // Services
    ros::ServiceServer push_viewport_service_server_;
    ros::ServiceServer empty_viewport_list_service_server_;
    ros::ServiceServer get_viewport_list_service_server_;
    ros::ServiceServer filter_viewport_depending_on_already_visited_viewports;

    ros::ServiceServer push_found_object_service_server_;
    ros::ServiceServer push_found_object_list_service_server_;
    ros::ServiceServer empty_found_object_list_service_server_;
    ros::ServiceServer get_found_object_list_service_server_;
    ros::ServiceServer visualize_sampled_poses_service_server_;

    ros::ServiceServer get_recognizer_name_service_server_;
    ros::ServiceServer get_intermediate_object_weight_service_server_;
    ros::ServiceServer get_all_objects_list_service_server_;
    ros::ServiceServer get_missing_object_list_service_server_;

    ros::ServiceServer get_complete_patterns_service_server_;
    ros::ServiceServer empty_complete_patterns_service_server_;
    ros::ServiceServer push_complete_patterns_service_server_;

    // the node handles
    ros::NodeHandle local_handle_;

    DebugHelperPtr debug_helper_ptr_;
    dynamic_reconfigure::Server<asr_world_model::DynamicParametersConfig> mDynamicReconfigServer;

    // handles
    boost::shared_ptr<ViewPortHandler> view_port_handler_ptr_;
    boost::shared_ptr<FoundObjectHandler> found_object_handler_ptr_;
    boost::shared_ptr<CommonInformationHandler> common_information_handler_ptr_;
    boost::shared_ptr<CompletePatternsHandler> complete_patterns_handler_ptr_;


    ModelTypePtrPerTypeMapPtr model_type_ptr_per_type_map_ptr_;

    SettingsPtr settings_ptr_;

public:
    /* ----------------- Public functions  ------------------  */
    /*!
     * \brief Creates a new instance of the world model and starts the services.
     */
    WorldModel();

private:

template<typename T>
static boost::shared_ptr<std::map<std::string, T>> processStringMap(std::string &map) {
   boost::shared_ptr<std::map<std::string, T>> returnMap = boost::shared_ptr<std::map<std::string, T>>(new std::map<std::string, T>());
   bool test = true;
   while(test) {
       std::size_t found = map.find(";");
       if (found != std::string::npos) {
           std::string temp;
           temp = map.substr(0,found);
           std::size_t temp_found = temp.find(",");
           (*returnMap)[temp.substr(0,temp_found)] = boost::lexical_cast<T>(temp.substr(temp_found+1));
           map = map.substr(found+1);
       } else {
          test = false;
           break;
       }
   }
   std::size_t temp_found = map.find(",");
   if (temp_found != std::string::npos) {
       (*returnMap)[map.substr(0,temp_found)] = boost::lexical_cast<T>(map.substr(temp_found+1));
   }
   return returnMap;
}

template<typename T>
static boost::shared_ptr<std::vector<std::pair<std::string, T>>> processStringVector(std::string &map) {
    boost::shared_ptr<std::vector<std::pair<std::string, T>>> returnVector =
            boost::shared_ptr<std::vector<std::pair<std::string, T>>>(new std::vector<std::pair<std::string, T>>);
    bool test = true;
    while(test) {
        std::size_t found = map.find(";");
        if (found != std::string::npos) {
            std::string temp;
            temp = map.substr(0,found);
            std::size_t temp_found = temp.find(",");
            returnVector->push_back(std::make_pair(temp.substr(0,temp_found), boost::lexical_cast<T>(temp.substr(temp_found+1))));
            map = map.substr(found+1);
        } else {
            test = false;
            break;
        }
    }
    std::size_t temp_found = map.find(",");
    if (temp_found != std::string::npos) {
        returnVector->push_back(std::make_pair(map.substr(0,temp_found), boost::lexical_cast<T>(map.substr(temp_found+1))));
    }
    return returnVector;
}

void dynamicReconfigureCallback(asr_world_model::DynamicParametersConfig &config, uint32_t level);

void checkParametersFromOtherNode();

void initParams();
void initMapsFromSqlTabel();
void initIntermediateObjectWeights(const ISM::TableHelperPtr &table_helper);
void initMapsFromWorldDescription();

void advertiseServices();

bool processEmptyFoundObjectListServiceCall(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

};

}
