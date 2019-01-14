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
        RobotGroupA(nh, ric_client) , roboteq_(nh), bms_(nh), dxl_motors_(nh),
        torso_control_(nh, "torso_joint")
{
    bms_.connect("/dev/mobilican/BMS");
    wheels_.reserve(2);
    wheels_[0] = "right_wheel_joint";
    wheels_[1] = "left_wheel_joint";
    roboteq_.connect("/dev/mobilican/ROBOTEQ", 115200, wheels_);
    roboteq_.getMotors(motors_);
    torso_control_.usePositionFilter(0.01);
}

void Armadillo_1::registerInterfaces() {

    // register arm motors
    dxl_motors_.registerHandles(joint_state_interface_,
                                position_interface_,
                                posvel_interface_);
    // register wheels
    for (roboteq::Motor * m : *motors_) {
        joint_state_interface_.registerHandle(m->joint_state_handle);
        velocity_interface_.registerHandle(m->joint_handle);
    }

    // register torso
    torso_control_.registerHandles(joint_state_interface_, effort_interface_);

    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_interface_);
    registerInterface(&posvel_interface_);
    registerInterface(&position_interface_);
    registerInterface(&effort_interface_);
}

void Armadillo_1::stop() {
    std::vector<roboteq::Motor*> * motors;
    roboteq_.getMotors(motors);
    for (roboteq::Motor * m : *motors) {
        m->stopMotor();
    }
}

void Armadillo_1::write(const ros::Time &time, const ros::Duration &duration) {
    roboteq_.write(time, duration);
    dxl_motors_.write();
}

void Armadillo_1::read(const ros::Time &time, const ros::Duration &duration) {
    roboteq_.read(time, duration);
    dxl_motors_.read();
    torsoRead(duration);
}

void Armadillo_1::torsoRead(const ros::Duration &duration) {
    if (got_torso_pos_) {
        torso_control_.update(current_torso_pos_, duration);
    }
}

void Armadillo_1::torsoWrite() {
    // add 1500 offset because torso limits in
    // armadillo2 xacro are b/w -500 - 500,
    // and ric servo get value b/w 1000-2000
    int command = torso_control_.output();
    command = Utils::clamp(command, -500, 500);
    ric_interface_ros::Servo servo_msg;
    servo_msg.value = command + 1500;
    servo_msg.id = TORSO_ID;
    ric_servo_pub_.publish(servo_msg);
}

void Armadillo_1::onProximityMsg(const ric_interface_ros::Proximity::ConstPtr &msg) {
    sensor_msgs::Range range_msg;
    range_msg.header.stamp = ros::Time::now();
    range_msg.min_range = 0.3;
    range_msg.max_range = 3.0;
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.range = msg->distance / 1000.0;
    range_msg.field_of_view = 0.7f;

    int urf_id = msg->id;

    switch (urf_id) {
        case UrfId::REAR:
            range_msg.header.frame_id = "rear_urf_link";
            urf_rear_pub_.publish(range_msg);
            break;
        case UrfId::RIGHT:
            range_msg.header.frame_id = "right_urf_link";
            urf_right_pub_.publish(range_msg);
            break;
        case UrfId::LEFT:
            range_msg.header.frame_id = "left_urf_link";
            urf_left_pub_.publish(range_msg);
            break;
        case TORSO_ID:
            double position = msg->distance / 1000.0; //convert to mm
            current_torso_pos_ = position;
            if (!got_torso_pos_) {
                torso_control_.initPositionFilter(current_torso_pos_);
                got_torso_pos_ = true;
            }
            break;
    }
}