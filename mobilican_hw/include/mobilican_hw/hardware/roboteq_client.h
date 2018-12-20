//
// Created by sub on 12/6/18.
//

#ifndef MOBILICAN_HW_ROBOTEQ_CLIENT_H
#define MOBILICAN_HW_ROBOTEQ_CLIENT_H

#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <roboteq/roboteq.h>
#include <roboteq/serial_controller.h>
#include <std_msgs/String.h>
#include <memory>
#include <mobilican_hw/utils.h>

class RoboteqClient {
private:

    ros::NodeHandle * nh_;
    std::string port_ = "/dev/ttyACM0";
    int32_t baud_ = 115200;
    roboteq::serial_controller * serial_ = nullptr;
    roboteq::Roboteq * interface_ = nullptr;

public:

    RoboteqClient(ros::NodeHandle &nh);
    ~RoboteqClient();
    void connect(const std::string & port, int32_t baud, const vector<std::string> & wheels_joints);
    void getMotors(std::vector<roboteq::Motor*> * motors) { interface_->getMotors(motors); }
    void write(const ros::Time &time, const ros::Duration& duration) { interface_->write(time, duration); }
    void read(const ros::Time &time, const ros::Duration& duration) { interface_->read(time, duration); }

};



#endif //MOBILICAN_HW_ROBOTEQ_CLIENT_H
