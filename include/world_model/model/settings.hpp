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

#include <boost/shared_ptr.hpp>
#include <vector>

namespace world_model {

    std::ostream& operator<<(std::ostream &strm, const std::vector<std::pair<std::string, std::string>> &objects_to_sample);

    struct Settings {
    public:
        //Sampling additional object poses for recognition result to consider pose uncertainty
        bool enable_object_sampling;
        bool calculate_deviations;
        int deviation_number_of_samples_position;
        int deviation_number_of_samples_orientation;
        std::vector<std::pair<std::string, std::string>> objects_to_sample;

        //Threshold Vars (Determines the maximum distance between two poses to be considered a neighbor)
        double object_position_distance_threshold;
        double object_orientation_rad_distance_threshold;
        double viewport_position_distance_threshold;
        double viewport_orientation_rad_distance_threshold;

        //The minimum number of neighbors that a cluster needs to considered for the best object list
        int object_rating_min_count;

        bool use_default_intermediate_object_weight;
        double default_intermediate_object_weight;

        std::string intermediate_object_weight_file_name;

        bool use_world_description;

        std::string dbfilename;
        double bin_size;
        double maxProjectionAngleDeviation;

        Settings() { }
        virtual ~Settings() { }
    };

    typedef boost::shared_ptr<Settings> SettingsPtr;


    std::ostream& operator<<(std::ostream &strm, const Settings &settings);
    std::ostream& operator<<(std::ostream &strm, const SettingsPtr &settings_ptr);
}



