
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

#ifndef MOBILICAN_HW_RIC_CLIENT_H
#define MOBILICAN_HW_RIC_CLIENT_H

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
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/String.h>
#include "ric_observer.h"
#include "mobilican_hw/hardware/hw_id.h"

#define RIC_DEAD_TIMEOUT            1.5 //secs

class RicClient {
public:
    typedef ric::protocol::keepalive::firmware_ver firm_ver;

private:

    bool got_keepalive_ = false;
    bool first_keepalive_ = true;
    bool is_connected_ = false;
    id_type hardware_id_ = -1;
    firm_ver firm_ver_;

    ros::Subscriber encoder_sub_,
            keepalive_sub_,
            orientation_sub_,
            proximity_sub_,
            logger_sub_,
            location_sub_,
            battery_sub_;


    ros::Publisher ric_servo_pub_;

    ros::ServiceClient terminate_ric_client_;

    ros::Timer keepalive_timer_;

    ros::NodeHandle *nh_ = nullptr;

    RicObserver* observer_ = nullptr;

    uint8_t hw_status_ = ric::protocol::package::Status::UNKNOWN;

    void onKeepAliveTimeout(const ros::TimerEvent &event);

    void onEncoderMsg(const ric_interface_ros::Encoder::ConstPtr& msg);
    void onKeepaliveMsg(const ric_interface_ros::Keepalive::ConstPtr& msg);
    void onOrientationMsg(const ric_interface_ros::Orientation::ConstPtr& msg);
    void onProximityMsg(const ric_interface_ros::Proximity::ConstPtr& msg);
    void onLoggerMsg(const ric_interface_ros::Logger::ConstPtr& msg);
    void onLocationMsg(const ric_interface_ros::Location::ConstPtr& msg);
    void onBatteryMsg(const ric_interface_ros::Battery::ConstPtr& msg);
    void onServoMsg(const ric_interface_ros::Servo::ConstPtr& msg);


public:


    RicClient(ros::NodeHandle &nh);
    void subscribe(RicObserver * observer) { observer_ = observer; }
    void waitForConnection(ros::Duration timeout);
    id_type getHardwareId() { return hardware_id_; }
    firm_ver getFirmwareVersion() { return firm_ver_; }

    void terminateRic();
    void writeServoCommand(uint16_t command, uint8_t id) const;
    bool isHwTestOk() { return (hw_status_ == ric::protocol::package::Status::OK); }
    bool isConnected() { return is_connected_; }
};




#endif //MOBILICAN_HW_RIC_CLIENT_H
