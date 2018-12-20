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


#include "mobilican_hw/hardware/ricboard/ric_client.h"

RicClient::RicClient(ros::NodeHandle &nh)
{
    nh_ = &nh;

    /* ric_interface_ros subscribers */
    encoder_sub_ = nh.subscribe("ric/encoder", 10, &RicClient::onEncoderMsg, this);
    keepalive_sub_ = nh.subscribe("ric/keepalive", 10, &RicClient::onKeepaliveMsg, this);
    orientation_sub_ = nh.subscribe("ric/orientation", 10, &RicClient::onOrientationMsg, this);
    proximity_sub_ = nh.subscribe("ric/proximity", 10, &RicClient::onProximityMsg, this);
    logger_sub_ = nh.subscribe("ric/logger", 10, &RicClient::onLoggerMsg, this);
    battery_sub_ = nh.subscribe("ric/battery", 10, &RicClient::onBatteryMsg, this);
    location_sub_ = nh.subscribe("ric/location", 10, &RicClient::onLocationMsg, this);
    ric_servo_pub_ = nh.advertise<ric_interface_ros::Servo>("ric/servo/cmd", 10);

    terminate_ric_client_ = nh.serviceClient<std_srvs::Trigger>("terminate_ric");

    keepalive_timer_ = nh.createTimer(ros::Duration(RIC_DEAD_TIMEOUT), &RicClient::onKeepAliveTimeout, this);
}

void RicClient::writeServoCommand(uint16_t command, uint8_t id) const
{
    if (command < 1000 || command > 2000)
        throw std::invalid_argument("command must be between 1000 and 2000");

    // send servo commands to ricboard
    ric_interface_ros::Servo servo_msg;

    servo_msg.id = id;
    servo_msg.value = command;
    ric_servo_pub_.publish(servo_msg);
}

void RicClient::onKeepAliveTimeout(const ros::TimerEvent &event)
{
    if (got_keepalive_)
        got_keepalive_ = false;
    else
    {
        is_connected_ = false;
        if (observer_ != nullptr)
            observer_->onKeepAliveTimeout();
    }
}


void RicClient::onKeepaliveMsg(const ric_interface_ros::Keepalive::ConstPtr& msg)
{
    // first keepalive indicate if hardware test failed
    if (first_keepalive_)
    {
        hardware_id_ = msg->hw_id;
        firm_ver_.major = msg->soft_major;
        firm_ver_.minor = msg->soft_minor;
        firm_ver_.patch = msg->soft_patch;
        first_keepalive_ = false;
    }
    hw_status_ = msg->status;
    got_keepalive_ = true;
    is_connected_ = true;
    if (observer_ != nullptr)
        observer_->onKeepAliveMsg(msg);
}

void RicClient::waitForConnection(ros::Duration timeout)
{
    ros::Time first_time = ros::Time::now();
    while (!isConnected() && (ros::Time::now() - first_time < timeout))
    {
        ros::Duration(1).sleep();
    }
}

void RicClient::terminateRic()
{
    std_srvs::Trigger kill_ric_srv;
    if (!terminate_ric_client_.call(kill_ric_srv))
        ROS_ERROR("calling ric_terminate service failed");
}

void RicClient::onLocationMsg(const ric_interface_ros::Location::ConstPtr &msg)
{
    if (observer_ != nullptr)
        observer_->onLocationMsg(msg);
}

void RicClient::onBatteryMsg(const ric_interface_ros::Battery::ConstPtr &msg)
{
    if (observer_ != nullptr)
        observer_->onBatteryMsg(msg);
}

void RicClient::onLoggerMsg(const ric_interface_ros::Logger::ConstPtr& msg)
{
    if (observer_ != nullptr)
        observer_->onLoggerMsg(msg);
}


void RicClient::onEncoderMsg(const ric_interface_ros::Encoder::ConstPtr& msg)
{
    if (observer_ != nullptr)
        observer_->onEncoderMsg(msg);
}

void RicClient::onOrientationMsg(const ric_interface_ros::Orientation::ConstPtr& msg)
{
    if (observer_ != nullptr)
        observer_->onOrientationMsg(msg);
}

void RicClient::onProximityMsg(const ric_interface_ros::Proximity::ConstPtr& msg)
{
    if (observer_ != nullptr)
        observer_->onProximityMsg(msg);
}
