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


#include "mobilican_hw/mobile_robot.h"

MobileRobot::MobileRobot(ros::NodeHandle & nh, RicClient & ric_client)
{
    nh_ = &nh;

    ric_client_ = &ric_client;

    ric_servo_pub_ = nh.advertise<ric_interface_ros::Servo>("ric/servo/cmd", 10);
    espeak_pub_ = nh.advertise<std_msgs::String>("/espeak_node/speak_line", 10);
    diagnos_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);

    if (ric_client_->isHwTestOk())
        ROS_INFO("hardware test ok");
    else
    {
        speak("hardware test failed");
        ROS_ERROR("hardware test failed. don't operate robot. "
                  "check diagnostics, and contact Robotican's support");
    }
}

void MobileRobot::onKeepAliveTimeout()
{
    ric_client_->terminateRic();
    speak("Rikboard disconnected, shutting down");
    Utils::terminateNode("Ricboard disconnected, shutting down");
}

void MobileRobot::speak(const char *msg) const
{
    std_msgs::String str_msg;
    str_msg.data = msg;
    espeak_pub_.publish(str_msg);
}

void MobileRobot::sendDiagnosticsMsg(const diagnostic_msgs::DiagnosticStatus &status) const
{
    diagnostic_msgs::DiagnosticArray diag_msg;
    diag_msg.header.frame_id="base_link";
    diag_msg.header.stamp=ros::Time::now();
    diag_msg.status.push_back(status);
    diagnos_pub_.publish(diag_msg);
}

void MobileRobot::onLoggerMsg(const ric_interface_ros::Logger::ConstPtr &msg) {
    switch(msg->sevirity)
    {
        case ric_interface_ros::Logger::INFO:
            ROS_INFO("ricboard says: %s", msg->message.c_str());
            break;
        case ric_interface_ros::Logger::WARN:
            ROS_WARN("ricboard says: %s", msg->message.c_str());
            break;
        case ric_interface_ros::Logger::CRITICAL:
            ROS_ERROR("ricboard says: %s", msg->message.c_str());
            break;
    }
}
