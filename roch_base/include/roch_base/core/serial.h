/**
*
*  File: HorizonProtocol.cpp
*  Desc: Generic serial communication functions. Pass in void pointers, let the
*        platform-specific implementation do the work. Usually, this should be
*        included by (and linked against) windows_serial.cpp or linux_serial.cpp
*  ����: ͨ�ô���ͨ�ź���,ͨ���ָ�룬ʵ���ض�ƽ̨����ͨ����
*	     ���Ӧ�ð���windows_serial.cpp��linux_serial.cpp(��֧��t��)
*   
*  Auth: R. Gariepy
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

#ifndef SERIAL_H_
#define SERIAL_H_

#include <stdint.h>
#include <stdio.h>

#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>  /* Malloc */
#include <assert.h>

namespace sawyer{
  class base_data{
  public:
    
    static const size_t MAX_MSG_LENGTH = 256;
     struct RawData{
      unsigned char data[MAX_MSG_LENGTH];
      int length;
      RawData():
      data(),length(0)
      { memset(data, 0xba, base_data::MAX_MSG_LENGTH);}
  }rawData;
  void clear(){memset(rawData.data, 0xba, MAX_MSG_LENGTH);rawData.length=0;}
  };
  class roch_driver: public base_data{
    
  public:
    
/**
* roch_driver singleton instance accessor.
* @return  The roch_driver singleton instance.
*/
  roch_driver &instance()
  {
    static roch_driver instance;
    return instance;
  }
  int OpenSerial(void **handle, const char *port_name);

  int SetupSerial(void *handle);

  int WriteData(void *handle, const char *buffer, int length);

  int ReadData(void *handle, char *buffer, int length);

  int CloseSerial(void *handle);
  base_data::RawData getGpInputData() const { 
     return rawData_.rawData; }
  base_data rawData_;
  
  };
};
#endif /* SERIAL_H_ */
