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


#ifndef LIZI_2_HW_H
#define LIZI_2_HW_H

#include <hardware_interface/joint_command_interface.h>
#include <sensor_msgs/BatteryState.h>
#include <boost/algorithm/clamp.hpp>
#include "mobilican_hw/robots_impl/robot_group_a.h"
#include "mobilican_hw/hardware/wheel/wheel.h"
#include "mobilican_hw/hardware/wheel/wheels_control.h"
#include "mobilican_hw/hardware/wheel/velocities_lpf.h"

class Lizi_2 : public RobotGroupA {

private:

    static constexpr float BATT_MAX = 16.7;
    static constexpr float BATT_MIN = 12.8;
    static constexpr float BATT_CELLS = 4;
    static constexpr uint16_t ENC_TICKS_PER_ROUND = 4480; //64 * 70

    int ric_servo_bias_ = 0;
    float control_loop_interval_ = 0;

    ros::NodeHandle *node_handle_;
    hardware_interface::VelocityJointInterface vel_joint_interface_;
    WheelsControl wheels_control_;
    VelocitiesLpf vels_lpf_;
    wheel front_right_wheel_;
    wheel front_left_wheel_;
    wheel rear_right_wheel_;
    wheel rear_left_wheel_;
    ros::Publisher battery_pub_;

    ros::Time prev_lpf_time_;

    ros::Timer vel_delta_timer_;

    static void updateWheelPosition(wheel &wheel, double new_pos);
    void onControlLoopTimer(const ros::TimerEvent &);

protected:

    void onEncoderMsg(const ric_interface_ros::Encoder::ConstPtr& msg) override;
    void onBatteryMsg(const ric_interface_ros::Battery::ConstPtr& msg) override;


public:

    Lizi_2(ros::NodeHandle & nh, RicClient & ric_client);

    void write(const ros::Time &time, const ros::Duration& duration) override {};
    void read(const ros::Time &time, const ros::Duration& duration) override {};
    void registerInterfaces() override;
    std::string getName() override { return "lizi_2"; };
    static id_type hwId() { return 0x68560102; }
};


#endif //LIZI_2_HW_H
