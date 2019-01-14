/*******************************************************************************
* Copyright (c) 2018 RoboTICan
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted (subject to the limitations in the disclaimer
* below) provided that the following conditions are met:
*
*     * Redistributions of source code must retain the above copyright notice,
*     this list of conditions and the following disclaimer.
*
*     * Redistributions in binary form must reproduce the above copyright
*     notice, this list of conditions and the following disclaimer in the
*     documentation and/or other materials provided with the distribution.
*
*     * Neither the name of the copyright holder nor the names of its
*     contributors may be used to endorse or promote products derived from this
*     software without specific prior written permission.
*
* NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
* THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
* CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
* PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
* CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
* EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
* PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
* IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Elhay Rauper*/


#ifndef MOBILICAN_HW_MOBILE_ROBOT_H
#define MOBILICAN_HW_MOBILE_ROBOT_H

#include <exception>
#include <std_msgs/String.h>
#include <ric_interface_ros/Encoder.h>
#include <ric_interface_ros/Keepalive.h>
#include <ric_interface_ros/Orientation.h>
#include <ric_interface_ros/Proximity.h>
#include <ric_interface_ros/Servo.h>
#include <ric_interface_ros/Toggle.h>
#include <ric_interface_ros/Logger.h>
#include <ric_interface_ros/Location.h>
#include <ric_interface_ros/Battery.h>
#include <ric_interface/protocol.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <tf/tf.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include "mobilican_hw/utils.h"
#include "mobilican_hw/hardware/ricboard/ric_client.h"
#include "mobilican_hw/hardware/ricboard/ric_observer.h"
#include "mobilican_hw/hardware/hw_id.h"

class MobileRobot : public hardware_interface::RobotHW, public RicObserver {

protected:

    ros::NodeHandle* nh_ = nullptr;

    hardware_interface::JointStateInterface joint_state_interface_;

    ros::Publisher ric_servo_pub_,
            espeak_pub_,
            diagnos_pub_;

    RicClient* ric_client_ = nullptr;


public:

    MobileRobot(ros::NodeHandle & nh, RicClient & ric_client);
    virtual ~MobileRobot() {};

    void sendDiagnosticsMsg(const diagnostic_msgs::DiagnosticStatus &status) const;

    // ric observer methods
    virtual void onEncoderMsg(const ric_interface_ros::Encoder::ConstPtr& msg) override {};
    virtual void onOrientationMsg(const ric_interface_ros::Orientation::ConstPtr& msg) override {};
    virtual void onProximityMsg(const ric_interface_ros::Proximity::ConstPtr& msg) override {};
    virtual void onLoggerMsg(const ric_interface_ros::Logger::ConstPtr& msg) override;
    virtual void onLocationMsg(const ric_interface_ros::Location::ConstPtr& msg) override {};
    virtual void onBatteryMsg(const ric_interface_ros::Battery::ConstPtr& msg) override {};
    virtual void onServoMsg(const ric_interface_ros::Servo::ConstPtr& msg) {};
    virtual void onKeepAliveMsg(const ric_interface_ros::Keepalive::ConstPtr& msg) override {};

    virtual void onKeepAliveTimeout() override;

    // general robot methods
    virtual void read(const ros::Time &time, const ros::Duration& duration) = 0;
    virtual void write(const ros::Time &time, const ros::Duration& duration) = 0;
    virtual void stop() = 0;

    virtual void registerInterfaces() = 0;
    virtual std::string getName() = 0;

    void speak(const char * msg) const;

};


#endif //MOBILICAN_HW_MOBILE_ROBOT_H
