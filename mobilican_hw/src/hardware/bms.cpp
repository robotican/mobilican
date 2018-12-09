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


#include "mobilican_hw/hardware/bms.h"


Bms::Bms(ros::NodeHandle &nh)
{
    nh_ = &nh;
}

void Bms::connect(const std::string &port)
{
    /* connect to batt FTDI */
    try
    {
        bms_.connect(port);
    }
    catch (bms::BMSException exp)
    {
        Utils::terminateNode(exp.what());
    }
    ROS_INFO("opened BMS port successfully \nport name: %s \nbaudrate: 9600", batt_port_.c_str());

    /* batt publisher */
    bat_pub_ = nh_->advertise<sensor_msgs::BatteryState>("battery", 10);
    bat_pub_timer_ = nh_->createTimer(ros::Duration(BATT_PUB_INTERVAL), &Bms::onBattPubTimer, this);
}

void Bms::setLowBatt(int val)
{
    if (val < 0 || val > 100)
        throw std::invalid_argument("battery percentage must be between 0-100");
    low_batt_ = val;
}


void Bms::onBattPubTimer(const ros::TimerEvent &event)
{
    bms::data bms_data;
    try
    {
        bms_data = bms_.read();
    }
    catch(bms::BMSErrorException exp)
    {
        Utils::terminateNode(exp.what());
    }
    catch(bms::BMSWarnException exp)
    {
        ROS_WARN("%s", exp.what());
    }

    sensor_msgs::BatteryState msg;
    msg.header.stamp = ros::Time::now();
    msg.present = true;
    msg.voltage = bms_data.vbat;
    msg.percentage = bms_data.soc;
    msg.current = bms_data.chrg_current - bms_data.dchrg_current;
    msg.charge = bms_data.chrg_current;
    msg.capacity = bms_data.cap_full; //Ah
    msg.power_supply_status = bms_data.is_chrg;
    msg.cell_voltage = bms_data.vcells;
    msg.location = "base_link";

    bat_pub_.publish(msg);

    /* if battery low and not in charging print warning */
    if ((low_batt_ >=0 && msg.percentage <= low_batt_) && !bms_data.is_chrg)
        ROS_WARN("[komodo2_hw/battery_pub]: LOW BATTERY, please connect komodo2 to charger");
}