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

#include "mobilican_hw/robots_impl/komodo_2.h"

/*
 * Komodo have only 2 motors and encoders, but because it have
 * 4 wheels, we have to simulate the front wheels so ROS can
 * use them currectly in joint_states
 */

Komodo_2::Komodo_2(ros::NodeHandle & nh, RicClient & ric_client) :
                        RobotGroupA(nh, ric_client) , roboteq_(nh), bms_(nh)
{
    bms_.connect("/dev/mobilican/BMS");

    // try connect to roboteq, and send real wheels to it
    real_wheels_.reserve(2);
    real_wheels_[0] = "right_rear_wheel_joint";
    real_wheels_[1] = "left_rear_wheel_joint";
    roboteq_.connect("/dev/mobilican/ROBOTEQ", 115200, real_wheels_);
    roboteq_.getMotors(motors_);

    virtual_wheels_[0].joint_name = "right_front_wheel_joint";
    virtual_wheels_[1].joint_name = "left_front_wheel_joint";
}

void Komodo_2::registerInterfaces()
{
    // register real motors and encoders (represented as rear wheels)
    for (roboteq::Motor * m : *motors_) {
        joint_state_interface_.registerHandle(m->joint_state_handle);
        vel_joint_interface_.registerHandle(m->joint_handle);
    }

    // register virtual wheels (represented as front wheels)
    for (int i=0; i<2; i++) {
        hardware_interface::JointStateHandle state_handle(virtual_wheels_[i].joint_name,
                                                          &virtual_wheels_[i].position,
                                                          &virtual_wheels_[i].velocity,
                                                          &virtual_wheels_[i].effort);
        joint_state_interface_.registerHandle(state_handle);
        hardware_interface::JointHandle joint_handle(joint_state_interface_.getHandle(virtual_wheels_[i].joint_name),
                                                     &virtual_wheels_[i].command_velocity);
        vel_joint_interface_.registerHandle(joint_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&vel_joint_interface_);
}

void Komodo_2::stop() {
    for (roboteq::Motor * m : *motors_) {
        m->stopMotor();
    }
}

void Komodo_2::write(const ros::Time &time, const ros::Duration &duration) {
    roboteq_.write(time, duration);
}

void Komodo_2::read(const ros::Time &time, const ros::Duration &duration) {
    roboteq_.read(time, duration);

    // simulate front virtual wheels be matching
    // their values to real rear wheels values
    for (int i=0; i<2; i++) {
        virtual_wheels_[i].position = (*motors_)[i]->position;
        virtual_wheels_[i].velocity = (*motors_)[i]->velocity;
        virtual_wheels_[i].effort = (*motors_)[i]->effort;
    }
}
