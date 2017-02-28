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

#include "world_model/model/model_object.hpp"

namespace world_model {

    struct ModelType {
    public:
        std::string type;
        std::string recognizerName;
        //Var for Intermediate Object ratings
        double weight;
        ModelObjectPtrPerIdMap model_object_ptr_per_id_map;

        ModelType(std::string type);
        virtual ~ModelType();
    };

    typedef boost::shared_ptr<ModelType> ModelTypePtr;
    typedef std::map<std::string, ModelTypePtr> ModelTypePtrPerTypeMap;
    typedef boost::shared_ptr<ModelTypePtrPerTypeMap> ModelTypePtrPerTypeMapPtr;


    std::ostream& operator<<(std::ostream &strm, const ModelType &model_object);
    std::ostream& operator<<(std::ostream &strm, const ModelTypePtr &model_object_ptr);

    std::ostream& operator<<(std::ostream &strm, const ModelTypePtrPerTypeMap &model_object_ptr_per_type_and_id_map);
    std::ostream& operator<<(std::ostream &strm, const ModelTypePtrPerTypeMapPtr &model_object_ptr_per_type_and_id_map_ptr);
}




