/**
*
*  \author     Paul Bovbel <pbovbel@sawyerrobotics.com>
*  \copyright  Copyright (c) 2014-2015, sawyer Robotics, Inc.
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
* Please send comments, questions, or patches to code@sawyerrobotics.com
*
*/

#ifndef roch_BASE_roch_DIAGNOSTICS_H
#define roch_BASE_roch_DIAGNOSTICS_H

#include "ros/ros.h"
#include "diagnostic_updater/diagnostic_updater.h"
#include "roch_base/core_wrapper.h"
#include "roch_msgs/RochStatus.h"

namespace roch_base
{

  class rochSoftwareDiagnosticTask :
    public diagnostic_updater::DiagnosticTask
  {
  public:
    explicit rochSoftwareDiagnosticTask(roch_msgs::RochStatus &msg, double target_control_freq);

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat);

    void updateControlFrequency(double frequency);

  private:
    void reset();

    double control_freq_, target_control_freq_;
    roch_msgs::RochStatus &msg_;
  };

  template<typename T>
  class rochHardwareDiagnosticTask :
    public diagnostic_updater::DiagnosticTask
  {
  public:
    explicit rochHardwareDiagnosticTask(roch_msgs::RochStatus &msg);

    void run(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
      typename core::Channel<T>::Ptr latest = core::Channel<T>::requestData(1.0);
      if (latest)
      {
        update(stat, latest);
      }
    }

    void update(diagnostic_updater::DiagnosticStatusWrapper &stat, typename core::Channel<T>::Ptr &status);

  private:
    roch_msgs::RochStatus &msg_;
  };

  template<>
  rochHardwareDiagnosticTask<sawyer::DataSystemStatus>::rochHardwareDiagnosticTask(roch_msgs::RochStatus &msg);

  template<>
  rochHardwareDiagnosticTask<sawyer::DataPowerSystem>::rochHardwareDiagnosticTask(roch_msgs::RochStatus &msg);

  template<>
  rochHardwareDiagnosticTask<sawyer::DataSafetySystemStatus>::rochHardwareDiagnosticTask(
    roch_msgs::RochStatus &msg);

  template<>
  void rochHardwareDiagnosticTask<sawyer::DataSystemStatus>::update(
    diagnostic_updater::DiagnosticStatusWrapper &stat,
    core::Channel<sawyer::DataSystemStatus>::Ptr &status);

  template<>
  void rochHardwareDiagnosticTask<sawyer::DataPowerSystem>::update(
    diagnostic_updater::DiagnosticStatusWrapper &stat,
    core::Channel<sawyer::DataPowerSystem>::Ptr &status);

  template<>
  void rochHardwareDiagnosticTask<sawyer::DataSafetySystemStatus>::update(
    diagnostic_updater::DiagnosticStatusWrapper &stat,
    core::Channel<sawyer::DataSafetySystemStatus>::Ptr &status);

}  // namespace roch_base
#endif  // roch_BASE_roch_DIAGNOSTICS_H
