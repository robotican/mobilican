

#include "mobilican_hw/mobile_robot.h"

MobileRobot::MobileRobot(ros::NodeHandle &nh, RicClient & ric_client)
{
    nh_ = &nh;
    ric_client_ = &ric_client;

    ric_client_->subscribe(this);

    ric_servo_pub_ = nh.advertise<ric_interface_ros::Servo>("ric/servo/cmd", 10);
    espeak_pub_ = nh.advertise<std_msgs::String>("/espeak_node/speak_line", 10);
    diagnos_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);
}

void MobileRobot::onKeepAliveTimeout()
{
    Utils::terminateNode("Ricboard disconnected, shutting down");
}

void MobileRobot::speak(const char *msg) const
{
    std_msgs::String str_msg;
    str_msg.data = msg;
    espeak_pub_.publish(str_msg);
}

MobileRobot* MobileRobot::buildRobot(ros::NodeHandle &nh,
                                     RicClient & ric_client,
                                     uint16_t hardware_id) const
{
    switch (hardware_id)
    {
        case (Lizi_2::getModelHardwareId()):
            return new Lizi_2(nh, ric_client);
    }
    return nullptr;
}



void MobileRobot::sendDiagnosticsMsg(const diagnostic_msgs::DiagnosticStatus &status) const
{
    diagnostic_msgs::DiagnosticArray diag_msg;
    diag_msg.header.frame_id="base_link";
    diag_msg.header.stamp=ros::Time::now();

    diag_msg.status.push_back(status);

    diagnos_pub_.publish(diag_msg);
}