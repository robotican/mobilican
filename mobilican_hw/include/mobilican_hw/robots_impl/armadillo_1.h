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

#ifndef MOBILICAN_HW_ARMADILLO_1_H
#define MOBILICAN_HW_ARMADILLO_1_H

#include "mobilican_hw/robots_impl/robot_group_a.h"
#include <hardware_interface/joint_command_interface.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <algorithm>
#include "mobilican_hw/hardware/bms.h"
#include "mobilican_hw/hardware/roboteq_client.h"
#include "mobilican_hw/hardware/dynamixel/dxl_motors_builder.h"
#include "mobilican_hw/hardware/effort_pos_control.h"


class Armadillo_1 : public RobotGroupA {

private:

    static const uint8_t TORSO_ID = 20;

    double current_torso_pos_ = 0;
    bool got_torso_pos_ = false;

    hardware_interface::PositionJointInterface position_interface_;
    hardware_interface::PosVelJointInterface posvel_interface_;
    hardware_interface::VelocityJointInterface velocity_interface_;
    hardware_interface::EffortJointInterface effort_interface_;

    RoboteqClient roboteq_;
    Bms bms_;
    DxlMotorsBuilder dxl_motors_;
    EffortPositionControl torso_control_;


    std::vector<roboteq::Motor*> * motors_ = nullptr;
    std::vector<std::string> wheels_;

    void onProximityMsg(const ric_interface_ros::Proximity::ConstPtr& msg) override;
    void torsoRead(const ros::Duration &duration);
    void torsoWrite();

public:

    Armadillo_1(ros::NodeHandle & nh, RicClient & ric_client);

    void write(const ros::Time &time, const ros::Duration& duration) override;
    void read(const ros::Time &time, const ros::Duration& duration) override;
    void registerInterfaces() override;
    std::string getName() override { return "armadillo_1"; };
    static id_type hwId() { return 0x68560301; }

};


#endif //MOBILICAN_HW_ARMADILLO_1_H
