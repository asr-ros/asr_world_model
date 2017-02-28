/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Hutmacher Robin, Karrenbauer Oliver, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Trautmann Jeremias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "world_model/complete_patterns_handler.hpp"

namespace world_model
{

CompletePatternsHandler::CompletePatternsHandler() : completePatterns() {
    debug_helper_ptr_ = DebugHelper::getInstance();
    ROS_INFO_STREAM("CompletePatternsHandler initialized");
}

bool CompletePatternsHandler::processPushCompletePatternsCall(asr_world_model::PushCompletePatterns::Request &request,
                                                              asr_world_model::PushCompletePatterns::Response &response) {
    debug_helper_ptr_->write(std::stringstream() << "Calling processPushCompletePatternsCall", (DebugHelper::SERVICE_CALLS + DebugHelper::COMPLETE_PATTERN));

    for (const asr_world_model::CompletePattern &completePattern : request.completePatterns) {
        CompletePatterns::iterator complete_it = completePatterns.find(completePattern.patternName);
        if (complete_it == completePatterns.end()) {
            completePatterns.insert(std::make_pair(completePattern.patternName, completePattern));
        } else {
            complete_it->second.confidence = completePattern.confidence;
        }
    }

    debug_helper_ptr_->write(std::stringstream() << "CompletePatterns after push:\n" << completePatterns, DebugHelper::COMPLETE_PATTERN);

    return true;
}

bool CompletePatternsHandler::processEmptyCompletePatternsServiceCall(asr_world_model::EmptyCompletePatterns::Request &request,
                                                              asr_world_model::EmptyCompletePatterns::Response &response) {
    debug_helper_ptr_->write(std::stringstream() << "Calling processEmptyCompletePatternsServiceCall", (DebugHelper::SERVICE_CALLS + DebugHelper::COMPLETE_PATTERN));

    completePatterns.clear();

    return true;
}

bool CompletePatternsHandler::processGetCompletePatternsServiceCall(asr_world_model::GetCompletePatterns::Request &request,
                                                                    asr_world_model::GetCompletePatterns::Response &response)
{
    debug_helper_ptr_->write(std::stringstream() << "Calling processGetCompletePatternsServiceCall", (DebugHelper::SERVICE_CALLS + DebugHelper::COMPLETE_PATTERN));

    for (const std::pair<std::string, asr_world_model::CompletePattern> &completePatternPair : completePatterns) {
        response.completePatterns.push_back(completePatternPair.second);
    }

    debug_helper_ptr_->write(std::stringstream() << "Returning completePatternsSet:\n" << response.completePatterns, DebugHelper::COMPLETE_PATTERN);

    return true;
}


std::ostream& operator<<(std::ostream &strm, const CompletePatterns &complete_patterns) {
    for (const std::pair<std::string, asr_world_model::CompletePattern> &completePatternPair : complete_patterns) {
        strm << completePatternPair.second;
    }
    return strm;
}

std::ostream& operator<<(std::ostream &strm, const std::vector<asr_world_model::CompletePattern> &complete_patterns) {
    for (const asr_world_model::CompletePattern &completePattern : complete_patterns) {
        strm << completePattern;
    }
    return strm;
}

}

