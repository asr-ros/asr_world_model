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
#include <set>


// ROS Main Inclusion
#include <ros/ros.h>

// Local Inclusion
#include "asr_world_model/CompletePattern.h"
#include "asr_world_model/EmptyCompletePatterns.h"
#include "asr_world_model/GetCompletePatterns.h"
#include "asr_world_model/PushCompletePatterns.h"

#include "world_model/helper/debug_helper.hpp"

namespace world_model
{

typedef std::map<std::string, asr_world_model::CompletePattern> CompletePatterns;

std::ostream& operator<<(std::ostream &strm, const CompletePatterns &complete_patterns);
std::ostream& operator<<(std::ostream &strm, const std::vector<asr_world_model::CompletePattern> &complete_patterns);

class CompletePatternsHandler
{
public:
    /* ----------------- Public members  ------------------  */
    // Wrapped Constants

    // PushCompletePatterns
    static const inline std::string GetPushCompletePatternsServiceName ()
    { return "push_complete_patterns"; }

    // EmptyCompletePatterns
    static const inline std::string GetEmptyCompletePatternsServiceName()
    { return "empty_complete_patterns"; }

    // GetCompletePatterns
    static const inline std::string GetGetCompletePatternsServiceName()
    { return "get_complete_patterns"; }

private:
    /* ----------------- Private members  ------------------  */

    CompletePatterns completePatterns;

    DebugHelperPtr debug_helper_ptr_;

public:
    /* ----------------- Public functions  ------------------  */
    /*!
     * \brief Creates a new instance of the CompletePatternsHandler
     */
    CompletePatternsHandler();

    bool processEmptyCompletePatternsServiceCall(asr_world_model::EmptyCompletePatterns::Request &request,
                                             asr_world_model::EmptyCompletePatterns::Response &response);

    bool processGetCompletePatternsServiceCall(asr_world_model::GetCompletePatterns::Request &request,
                                           asr_world_model::GetCompletePatterns::Response &response);

    bool processPushCompletePatternsCall(asr_world_model::PushCompletePatterns::Request &request,
                                        asr_world_model::PushCompletePatterns::Response &response);

};

}

