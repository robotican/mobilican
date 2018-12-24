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


#ifndef MOBILICAN_HW_OUTDOOR_ROBOT_H
#define MOBILICAN_HW_OUTDOOR_ROBOT_H

#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include "mobilican_hw/mobile_robot.h"


class RobotGroupA : public MobileRobot {

protected:

    enum UrfId { REAR = 10, RIGHT = 11, LEFT = 12 };

    ros::Publisher urf_rear_pub_;
    ros::Publisher urf_right_pub_;
    ros::Publisher urf_left_pub_;
    ros::Publisher imu_pub_;
    ros::Publisher mag_pub_;
    ros::Publisher gps_pub_;

public:

    RobotGroupA(ros::NodeHandle & nh, RicClient & ric_client);
    virtual ~RobotGroupA() {};

    // ric observer methods
    virtual void onOrientationMsg(const ric_interface_ros::Orientation::ConstPtr& msg) override;
    virtual void onLocationMsg(const ric_interface_ros::Location::ConstPtr& msg) override;
    virtual void onProximityMsg(const ric_interface_ros::Proximity::ConstPtr& msg) override;

    // general robot methods
    virtual void read(const ros::Time &time, const ros::Duration& duration) = 0;
    virtual void write(const ros::Time &time, const ros::Duration& duration) = 0;

    virtual void registerInterfaces() = 0;
    virtual std::string getName() = 0;
};


#endif //MOBILICAN_HW_OUTDOOR_ROBOT_H
