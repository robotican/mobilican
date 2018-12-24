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

#include "mobilican_hw/hardware/effort_pos_control.h"

EffortPositionControl::EffortPositionControl(ros::NodeHandle &nh, std::string joint_name) {
    nh_ = &nh;
    joint_name_ = joint_name;
}

void EffortPositionControl::usePositionFilter(float alpha) {
    use_position_filter_ = true;
    pos_filter_alpha_ = alpha;
}

void EffortPositionControl::registerHandles(hardware_interface::JointStateInterface &joint_state_interface,
                                hardware_interface::EffortJointInterface &effort_interface)
{
    joint_state_handles_.push_back(hardware_interface::JointStateHandle (joint_name_,
                                                                         &position_,
                                                                         &velocity_,
                                                                         &effort_));
    joint_state_interface.registerHandle(joint_state_handles_.back());

    /* joint command registration */
    pos_handles_.push_back(hardware_interface::JointHandle (joint_state_interface.getHandle(joint_name_),
                                                            &command_effort_));
    effort_interface.registerHandle(pos_handles_.back());
}

void EffortPositionControl::initPositionFilter(double init_position) {
    try{
        pos_filter_.init(
                ros::NodeHandle(*nh_, std::string(joint_name_ + "/lpf")),
                pos_filter_alpha_,
                init_position
        );
    } catch (std::invalid_argument exp) {
        ROS_ERROR("[lizi_hw/velocities_lpf]: wheel %s error: %s. shutting down.",
                  joint_name_.c_str(),
                  exp.what());
        Utils::terminateNode("shutting down");
    }
}

void EffortPositionControl::update(double position, const ros::Duration elapsed) {
    position_ = position;
    if (use_position_filter_) {
        if (first_read_) {
            initPositionFilter(position_);
            first_read_ = false;
        }
        position_ = pos_filter_.filter(position_, 0);
    }
    velocity_ = (position_ - prev_position_) / elapsed.sec;
    effort_ = command_effort_;
    prev_position_ = position;
}