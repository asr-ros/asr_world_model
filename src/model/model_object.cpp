/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Hutmacher Robin, Karrenbauer Oliver, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Trautmann Jeremias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "world_model/model/model_object.hpp"
#include <ostream>

namespace world_model {
    ModelObject::ModelObject(std::string id) : id(id), objectCount(0), bestHypotheses(), allFoundHypotheses() { }
    ModelObject::~ModelObject() { }


    std::ostream& operator<<(std::ostream &strm, const PbdObjectCluster &pbd_object_cluster) {
        strm << "AsrObject:" << '\n';
        strm << pbd_object_cluster.first;
        strm << "with neighborhood count: " << pbd_object_cluster.second << '\n';
        return strm;
    }

    std::ostream& operator<<(std::ostream &strm, const PbdObjectClusterList &pbd_object_cluster_list) {
        for (const PbdObjectCluster &pbd_object_cluster : pbd_object_cluster_list) {
            strm << pbd_object_cluster;
        }
        return strm;
    }


    std::ostream& operator<<(std::ostream &strm, const ModelObject &model_object) {
        strm << '\t' << '\t' << "id: " << model_object.id << '\n';
        strm << '\t' << '\t' << "objectCount: " << model_object.objectCount << '\n';

        strm << '\t' << '\t' << "bestHypotheses:" << '\n';
        strm << model_object.bestHypotheses;

        strm << '\t' << '\t' << "allFoundHypotheses:" << '\n';
        strm << model_object.allFoundHypotheses;

        return strm;
    }

    std::ostream& operator<<(std::ostream &strm, const ModelObjectPtr &model_object_ptr) {
        return strm << *model_object_ptr;
    }

    std::ostream& operator<<(std::ostream &strm, const ModelObjectPtrPerIdMap &model_object_ptr_per_id_map) {
        for (const std::pair<std::string, ModelObjectPtr> &model_object_ptr_per_id_pair : model_object_ptr_per_id_map) {
            strm << '\t' << "ModelObject:" << '\n';
            strm << model_object_ptr_per_id_pair.second;
        }
        return strm;
    }
}
