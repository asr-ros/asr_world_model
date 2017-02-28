/**

Copyright (c) 2016, Aumann Florian, Borella Jocelyn, Hutmacher Robin, Karrenbauer Oliver, Meißner Pascal, Schleicher Ralf, Stöckle Patrick, Trautmann Jeremias
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "world_model/helper/debug_helper.hpp"

namespace world_model {

boost::shared_ptr<DebugHelper> DebugHelper::instancePtr;

boost::shared_ptr<DebugHelper> DebugHelper::getInstance() {
    if (!instancePtr)
        instancePtr = boost::shared_ptr<DebugHelper>(new DebugHelper());
    return instancePtr;
}

void DebugHelper::resetInstance() {
    if (instancePtr) {
        instancePtr->setLevels();
    }
}

DebugHelper::DebugHelper() {
    this->setLevels();
}

void DebugHelper::write(const char *text, const unsigned int &levels) const
{
    if (this->checkLevel(levels)) {
        ROS_DEBUG_STREAM(getDebugLevelString(levels) << ": " << text);
    }
}

void DebugHelper::write(const std::string& text, const unsigned int &levels) const {
    const char* cText = text.c_str();

    this->write(cText, levels);
}

void DebugHelper::write(const std::ostream& text, const unsigned int &levels) const {
    std::stringstream ss;
    ss << text.rdbuf();
    std::string s = ss.str();

    this->write(s, levels);
}

void DebugHelper::writeNoticeably(const char *text, const unsigned int &levels) const
{
    if (this->checkLevel(levels)) {
        ROS_DEBUG_STREAM(" ");
        ROS_DEBUG_STREAM(getDebugLevelString(levels) << ": " << text);
        ROS_DEBUG_STREAM(" ");
    }
}

void DebugHelper::writeNoticeably(const std::string &text, const unsigned int &levels) const
{
    const char* cText = text.c_str();

    this->writeNoticeably(cText, levels);
}

void DebugHelper::writeNoticeably(const std::ostream &text, const unsigned int &levels) const
{
    std::stringstream ss;
    ss << text.rdbuf();
    std::string s = ss.str();

    this->writeNoticeably(s, levels);
}

unsigned int DebugHelper::getLevel() const
{
    return mLevels;
}

std::string DebugHelper::getLevelString() const
{
    if (mLevels == ALL)
        return "ALL";
    if (mLevels == NONE)
        return "NONE";

    std::string level = "";

    if (mLevels & PARAMETERS) {
        addToString(level, "PARAMETERS");
    }
    if (mLevels & SERVICE_CALLS) {
        addToString(level, "SERVICE_CALLS");
    }
    if (mLevels & COMMON_INFORMATION) {
        addToString(level, "COMMON_INFORMATION");
    }
    if (mLevels & FOUND_OBJECT) {
        addToString(level, "FOUND_OBJECT");
    }
    if (mLevels & VIEW_PORT) {
        addToString(level, "VIEW_PORT");
    }
    if (mLevels & COMPLETE_PATTERN) {
        addToString(level, "COMPLETE_PATTERN");
    }

    return level;
}

std::string DebugHelper::getDebugLevelString(const unsigned int &levels) const
{
    std::string level = "";

    if (levels & PARAMETERS)
        addToString(level, "PARAMETERS");
    if (levels & SERVICE_CALLS)
        addToString(level, "SERVICE_CALLS");
    if (levels & COMMON_INFORMATION)
        addToString(level, "COMMON_INFORMATION");
    if (levels & FOUND_OBJECT)
        addToString(level, "FOUND_OBJECT");
    if (levels & VIEW_PORT)
        addToString(level, "VIEW_PORT");
    if (levels & COMPLETE_PATTERN)
        addToString(level, "COMPLETE_PATTERN");

    return level;
}

bool DebugHelper::checkLevel(const unsigned int &levels) const {
    return levels & mLevels;
}

void DebugHelper::setLevels() {
    ros::NodeHandle mNodeHandle = ros::NodeHandle(ros::this_node::getName());

    std::vector<std::string> debugLevels;
    mNodeHandle.getParam("debugLevels", debugLevels);

    if (debugLevels.size() == 0) {
        // Parse debugLevels, because there is no vector in dynamic_reconfigure
        std::string levelString;
        mNodeHandle.getParam("debugLevels", levelString);

        levelString.erase(std::remove(levelString.begin(), levelString.end(), '['), levelString.end());
        levelString.erase(std::remove(levelString.begin(), levelString.end(), ']'), levelString.end());
        levelString.erase(std::remove(levelString.begin(), levelString.end(), ' '), levelString.end());

        std::stringstream ss(levelString);
        std::string item;
        while (std::getline(ss, item, ',')) {
            debugLevels.push_back(item);
        }
    }

    mLevels = parseLevels(debugLevels);
}

int DebugHelper::parseLevels(const std::vector<std::string> &levels) {
    if (levels.size() == 0)
        return ALL;
    if (levels.size() == 1) {
        if (levels.at(0).compare("ALL") == 0)
            return ALL;
        if (levels.at(0).compare("NONE") == 0)
            return NONE;
    }

    int level = 0;
    for (unsigned int i = 0; i < levels.size(); ++i) {
        if (levels.at(i).compare("PARAMETERS") == 0) {
            level += PARAMETERS;
        }
        else if (levels.at(i).compare("SERVICE_CALLS") == 0) {
            level += SERVICE_CALLS;
        }
        else if (levels.at(i).compare("COMMON_INFORMATION") == 0) {
            level += COMMON_INFORMATION;
        }
        else if (levels.at(i).compare("FOUND_OBJECT") == 0) {
            level += FOUND_OBJECT;
        }
        else if (levels.at(i).compare("VIEW_PORT") == 0) {
            level += VIEW_PORT;
        }
        else if (levels.at(i).compare("COMPLETE_PATTERN") == 0) {
            level += COMPLETE_PATTERN;
        }
        else {
            ROS_ERROR_STREAM("Invalid debug level: " << levels.at(i));
            throw "Invalid debug level";
        }
    }

    return level;
}

void DebugHelper::addToString(std::string& s, const std::string& add)
{
    if (s.size() != 0)
        s += ", ";
    s += add;
}

}
