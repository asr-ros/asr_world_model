/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Hutmacher Robin, Karrenbauer Oliver, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Trautmann Jeremias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "world_model/model/settings.hpp"
#include <ostream>

namespace world_model {

std::ostream& operator<<(std::ostream &strm, const std::vector<std::pair<std::string, std::string>> &objects_to_sample) {
    strm << "\tobjects_to_sample:\n";
    for (const std::pair<std::string, std::string> &object_type_to_id : objects_to_sample) {
        strm << "\t\ttype: " << object_type_to_id.first << " id: " << object_type_to_id.second << "\n";
    }

    return strm;
}


std::ostream& operator<<(std::ostream &strm, const Settings &settings) {
    strm << '\t' << "enable_object_sampling: " << settings.enable_object_sampling << '\n';
    strm << '\t' << "calculate_deviations: " << settings.calculate_deviations << '\n';
    strm << '\t' << "deviation_number_of_samples_position: " << settings.deviation_number_of_samples_position << '\n';
    strm << '\t' << "deviation_number_of_samples_orientation: " << settings.deviation_number_of_samples_orientation << '\n';
    strm << settings.objects_to_sample;

    strm << '\t' << "object_position_distance_threshold: " << settings.object_position_distance_threshold << '\n';
    strm << '\t' << "object_orientation_rad_distance_threshold: " << settings.object_orientation_rad_distance_threshold << '\n';
    strm << '\t' << "viewport_position_distance_threshold: " << settings.viewport_position_distance_threshold << '\n';
    strm << '\t' << "viewport_orientation_rad_distance_threshold: " << settings.viewport_orientation_rad_distance_threshold << '\n';

    strm << '\t' << "object_rating_min_count: " << settings.object_rating_min_count << '\n';

    strm << '\t' << "use_default_intermediate_object_weight: " << settings.use_default_intermediate_object_weight << '\n';
    strm << '\t' << "default_intermediate_object_weight: " << settings.default_intermediate_object_weight << '\n';

    strm << '\t' << "intermediate_object_weight_file_name: " << settings.intermediate_object_weight_file_name << '\n';

    strm << '\t' << "use_world_description: " << settings.use_world_description << '\n';

    strm << '\t' << "dbfilename: " << settings.dbfilename << '\n';
    strm << '\t' << "bin_size: " << settings.bin_size << '\n';
    strm << '\t' << "maxProjectionAngleDeviation: " << settings.maxProjectionAngleDeviation << '\n';
    return strm;
}

std::ostream& operator<<(std::ostream &strm, const SettingsPtr &settings_ptr) {
    return strm << *settings_ptr;
}
}
