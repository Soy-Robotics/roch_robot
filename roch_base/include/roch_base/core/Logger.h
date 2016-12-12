/**
*
*  File: Logger.h
*  Desc: Provides the Logger singleton which is used within the sawyer API
*        for log / trace message control
*  ����: �ṩ��b��־,������Ϣ���Ƶ���־������Ϣ���
*  Auth: Iain Peet
*
*  Copyright (c) 2010, sawyer Robotics, Inc.
*  All Rights Reserved
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*     * Neither the name of sawyer Robotics, Inc. nor the
*       names of its contributors may be used to endorse or promote products
*       derived from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL sawyer ROBOTICS, INC. BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* Please send comments, questions, or patches to skynet@sawyerrobotics.com
*
*/

#ifndef CPR_LOGGER_H
#define CPR_LOGGER_H

#include <iostream>

namespace sawyer
{

  class Logger
  {
  private:
    bool enabled;
    int level;

    std::ostream *stream;

    std::ofstream *nullStream; //i.e /dev/null

  public:
    enum logLevels
    {
      ERROR_LEV,
      EXCEPTION,
      WARNING,
      INFO,
      DETAIL
    };
    static const char *levelNames[]; // strings indexed by enumeration. 

  private:
    Logger();

    ~Logger();

    void close();

  public:
    static Logger &instance();

    std::ostream &entry(enum logLevels level, const char *file = 0, int line = -1);

    void setEnabled(bool enabled);

    void setLevel(enum logLevels newLevel);

    void setStream(std::ostream *stream);

    void hookFatalSignals();

    friend void loggerTermHandler(int signal);
  };

  void loggerTermHandler(int signal);

}; // namespace sawyer

// convenience macros
#define CPR_LOG(level) (sawyer::Logger::instance().entry((level), __FILE__, __LINE__ ))
#define CPR_ERR()      CPR_LOG(sawyer::Logger::ERROR)
#define CPR_EXCEPT()   (sawyer::Logger::instance().entry(sawyer::Logger::EXCEPTION))
#define CPR_WARN()     CPR_LOG(sawyer::Logger::WARNING)
#define CPR_INFO()     CPR_LOG(sawyer::Logger::INFO)
#define CPR_DTL()      CPR_LOG(sawyer::Logger::DETAIL)

#endif //CPR_LOGGER_H

