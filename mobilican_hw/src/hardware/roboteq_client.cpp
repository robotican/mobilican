//
// Created by sub on 12/6/18.
//

#include "mobilican_hw/hardware/roboteq_client.h"

RoboteqClient::~RoboteqClient()
{
    serial_->stop();
    if (serial_ != nullptr)
    {
        delete serial_;
        delete interface_;
    }
}

RoboteqClient::RoboteqClient(ros::NodeHandle &nh)
{
    nh_ = &nh;
}

void RoboteqClient::connect(const std::string &port,
        int32_t baud,
        const vector<std::string> & wheels_joints)
{
    ROS_INFO_STREAM("Try to connect to roboteq on"
                    "\nport: " << port <<
                    "\nbaud: " << baud);
    serial_ = new roboteq::serial_controller(port, baud);
    try
    {
        if (serial_->start())
            ROS_INFO_STREAM("Roboteq serial port opened successfully");
        else
        {
            Utils::terminateNode("Failed to open roboteq serial port");
        }
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Failed to open roboteq port: " << e.what());
    }

    interface_ = new roboteq::Roboteq(*nh_, *nh_, serial_, wheels_joints);
    interface_->initialize();
}

