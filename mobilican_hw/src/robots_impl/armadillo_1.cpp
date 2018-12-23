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


#include "mobilican_hw/robots_impl/armadillo_1.h"

Armadillo_1::Armadillo_1(ros::NodeHandle &nh, RicClient &ric_client) :
        MobileRobot(nh, ric_client) , roboteq_(nh), bms_(nh), dxl_motors_(nh) {

    bms_.connect("/dev/mobilican/BMS");
    wheels_.reserve(2);
    wheels_[0] = "right_wheel_joint";
    wheels_[1] = "left_wheel_joint";
    roboteq_.connect("/dev/mobilican/ROBOTEQ", 115200, wheels_);
    roboteq_.getMotors(motors_);
}

void Armadillo_1::registerInterfaces() {

    // register arm motors
    dxl_motors_.registerHandles(joint_state_interface_,
                                position_interface_,
                                posvel_interface_);

    //TODO: build torso class that close loop and register joint_state_interface and effort_interface

    // register wheels
    for (roboteq::Motor * m : *motors_) {
        joint_state_interface_.registerHandle(m->joint_state_handle);
        velocity_interface_.registerHandle(m->joint_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_interface_);
    registerInterface(&posvel_interface_);
    registerInterface(&position_interface_);
    registerInterface(&velocity_interface_);
    registerInterface(&effort_interface_);
}

void Armadillo_1::write(const ros::Time &time, const ros::Duration &duration) {
    roboteq_.write(time, duration);
    dxl_motors_.write();
}

void Armadillo_1::read(const ros::Time &time, const ros::Duration &duration) {
    roboteq_.read(time, duration);
    dxl_motors_.read()
}