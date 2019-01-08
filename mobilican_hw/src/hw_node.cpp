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


#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <memory>
#include "mobilican_hw/mobile_robot.h"
#include "mobilican_hw/hardware/ricboard/ric_client.h"
#include "mobilican_hw/utils.h"
#include "mobilican_hw/mobile_robot.h"
#include "mobilican_hw/robots_factory.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "lizi_hw_node");
    ros::NodeHandle nh;
    ros::AsyncSpinner async_spinner(2);
    async_spinner.start();

    RicClient ric_client(nh);
    ROS_INFO("waiting for signal from ricboard...");
    ric_client.waitForConnection(ros::Duration(5));
    id_type hw_id = -1;
    RicClient::firm_ver firm_ver;
    if (!ric_client.isConnected()) {
        Utils::terminateNode("Didn't get any signal from ricboard. "
                             "Make sure ric_interface_node is running");
    } else {
        hw_id = ric_client.getHardwareId();
        firm_ver = ric_client.getFirmwareVersion();
        ROS_INFO_STREAM("detected hardware id: " << hw_id);
        ROS_INFO_STREAM("detected firmware version: "
               << (int)firm_ver.major << "."
               << (int)firm_ver.minor << "."
               << (int)firm_ver.patch);
    }
    MobileRobot * robot = RobotsFactory::build(nh, hw_id, ric_client);
    ROS_INFO_STREAM("loaded " << robot->getName() << " robot");
    if (robot == nullptr) {
        Utils::terminateNode("invalid hardware hw id");
    }
    robot->registerInterfaces();
    ric_client.subscribe(*robot);
    controller_manager::ControllerManager controller_manager(robot);
    ros::Time last_time = ros::Time::now();
    while (ros::ok()) {
        ros::Duration duration = ros::Time::now() - last_time;
        last_time = ros::Time::now();
        robot->read(ros::Time::now(), duration);
        ros::Duration(0.005).sleep();
        controller_manager.update(ros::Time::now(), duration);
        robot->write(ros::Time::now(), duration);
        ros::Duration(0.005).sleep();
    }

    delete robot;
}
