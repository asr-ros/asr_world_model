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

#include <ros/ros.h>
#include <string>
#include <sstream>
#include <vector>

namespace world_model {

/*!
 * \brief The DebugHelper class is responsible for debug output.
 */
class DebugHelper {

public:

    enum DebugLevel {
        PARAMETERS = 1,
        SERVICE_CALLS = 2,
        COMMON_INFORMATION = 4,
        FOUND_OBJECT = 8,
        VIEW_PORT = 16,
        COMPLETE_PATTERN = 32
    };

private:

    static boost::shared_ptr<DebugHelper> instancePtr;
    static const int ALL = PARAMETERS + SERVICE_CALLS + COMMON_INFORMATION + FOUND_OBJECT
                            + VIEW_PORT + COMPLETE_PATTERN;
    static const int NONE = 0;

    unsigned int mLevels;

public:

    static boost::shared_ptr<DebugHelper> getInstance();

    static void resetInstance();

    /*!
     * \brief writes the text to the console if it has a level that allows it
     * \param text the text
     * \param level the debug level of the text
     */
    void write(const char * text, const unsigned int &levels) const;

    /*!
     * \brief writes the text to the console if it has a level that allows it
     * \param text the text
     * \param level the debug level of the text
     */
    void write(const std::string &text, const unsigned int &levels) const;

    /*!
     * \brief writes the text to the console if it has a level that allows it
     * \param text the text
     * \param level the debug level of the text
     */
    void write(const std::ostream &text, const unsigned int &levels) const;

    /*!
     * \brief writes the text noticeably to the console if it has a level that allows it
     * \param text the text
     * \param level the debug level of the text
     */
    void writeNoticeably(const char * text, const unsigned int &levels) const;

    /*!
     * \brief writes the text noticeably to the console if it has a level that allows it
     * \param text the text
     * \param level the debug level of the text
     */
    void writeNoticeably(const std::string &text, const unsigned int &levels) const;

    /*!
     * \brief writes the text noticeably to the console if it has a level that allows it
     * \param text the text
     * \param level the debug level of the text
     */
    void writeNoticeably(const std::ostream &text, const unsigned int &levels) const;

    /*!
     * \brief returns the debug level that is set
     * \return the debug level that is set.
     */
    unsigned int getLevel() const;

    /*!
     * \brief returns the debug levels that are set as string
     * \return the debug levels that are set as string.
     */
    std::string getLevelString() const;


    /*!
     * \brief sets the allowed debug levels
     */
    void setLevels();

private:

    DebugHelper();

    /*!
     * \brief parses the level list to the corresponding integer
     * \param levels the list of level strings
     * \return the corresponding integer
     */
    static int parseLevels(const std::vector<std::string> &levels);

    /*!
     * \brief adds a string to a given string s. Puts a comma between them if the string s has a size bigger than 0.
     * \param s [in,out] the string
     * \param add [in] the string to add
     */
    static void addToString(std::string& s, const std::string& add);

    /*!
     * \brief checks whether the given level is allowed
     * \param level the level
     * \return whether the given level is allowed.
     * true if it is allowed and false if it is not.
     */
    bool checkLevel(const unsigned int &levels) const;

    /*!
     * \brief returns the name of the given DebugLevel
     * \param level the level
     * \return the name of the given DebugLevel
     */
    std::string getDebugLevelString(const unsigned int &levels) const;

};

typedef boost::shared_ptr<DebugHelper> DebugHelperPtr;
}
