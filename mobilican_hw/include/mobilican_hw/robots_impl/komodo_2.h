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

#ifndef MOBILICAN_HW_KOMODO_2_H
#define MOBILICAN_HW_KOMODO_2_H

#include "mobilican_hw/mobile_robot.h"
#include <hardware_interface/joint_command_interface.h>
#include "mobilican_hw/hardware/roboteq_client.h"
#include "mobilican_hw/hardware/wheel/wheel.h"


#define KOMODO_2_HW_ID                12

class Komodo_2 : public MobileRobot
{

private:

    hardware_interface::VelocityJointInterface vel_joint_interface_;

    RoboteqClient roboteq_;

    wheel virtual_wheels_[2];
    std::vector<roboteq::Motor*> * motors_ = nullptr;

protected:

    void onEncoderMsg(const ric_interface_ros::Encoder::ConstPtr& msg) override {};
    void onOrientationMsg(const ric_interface_ros::Orientation::ConstPtr& msg) override {};
    void onProximityMsg(const ric_interface_ros::Proximity::ConstPtr& msg) override {};
    void onLoggerMsg(const ric_interface_ros::Logger::ConstPtr& msg) override {};
    void onLocationMsg(const ric_interface_ros::Location::ConstPtr& msg) override {};
    void onBatteryMsg(const ric_interface_ros::Battery::ConstPtr& msg) override {};


public:

    Komodo_2(ros::NodeHandle & nh, RicClient & ric_client);

    void write(const ros::Time &time, const ros::Duration& duration) override;
    void read(const ros::Time &time, const ros::Duration& duration) override;
    void registerInterfaces() override;
    std::string getName() override { return "komodo_2"; };
};


#endif //MOBILICAN_HW_KOMODO_2_H
