

#include "mobilican_hw/mobile_robot.h"

MobileRobot::MobileRobot(ros::NodeHandle &nh, RicClient ric_client)
{
    nh_ = &nh;

    ric_client_ = &ric_client;
    ric_client_->subscribe(this);

    ric_servo_pub_ = nh.advertise<ric_interface_ros::Servo>("ric/servo/cmd", 10);
    espeak_pub_ = nh.advertise<std_msgs::String>("/espeak_node/speak_line", 10);
    diagnos_pub_ = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics", 10);

    if (!ric_client_->isHwTestOk())
        ROS_INFO("Hardware test ok");
    else
    {
        speak("hardware test failed");
        ROS_ERROR("Hardware test failed. Don't operate robot. "
                  "Check diagnostics, and contact Robotican's support");
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