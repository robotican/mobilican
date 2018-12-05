//
// Created by sub on 12/4/18.
//

#ifndef MOBILICAN_HW_MOBILE_ROBOT_H
#define MOBILICAN_HW_MOBILE_ROBOT_H

#include <exception>
#include <std_msgs/String.h>
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
#include <hardware_interface/robot_hw.h>
#include "mobilican_hw/robots_impl/lizi_2/lizi_2.h"
#include "mobilican_hw/utils.h"
#include "mobilican_hw/hardware/ric_client.h"


#define RIC_DEAD_TIMEOUT            1.5 //secs


class MobileRobot : public hardware_interface::RobotHW
{

private:

//    ros::NodeHandle* nh_ = nullptr;
//    ros::Publisher ric_servo_pub_,
//            espeak_pub_,
//            diagnos_pub_;

public:
//
//    MobileRobot(ros::NodeHandle & nh);
//
//    void sendDiagnosticsMsg(const diagnostic_msgs::DiagnosticStatus &status) const;
//
//    // ric observer methods
//    virtual void onEncoderMsg(const ric_interface_ros::Encoder::ConstPtr& msg) = 0;
//    virtual void onOrientationMsg(const ric_interface_ros::Orientation::ConstPtr& msg) = 0;
//    virtual void onProximityMsg(const ric_interface_ros::Proximity::ConstPtr& msg) = 0;
//    virtual void onLoggerMsg(const ric_interface_ros::Logger::ConstPtr& msg) = 0;
//    virtual void onLocationMsg(const ric_interface_ros::Location::ConstPtr& msg) = 0;
//    virtual void onBatteryMsg(const ric_interface_ros::Battery::ConstPtr& msg) = 0;
//    virtual void onKeepAliveTimeout() = 0;
//
//    // general robot methods
//    virtual void read(const ros::Time &time, const ros::Duration& duration) = 0;
//    virtual void write(const ros::Time &time, const ros::Duration& duration) = 0;
//
//    virtual void registerInterfaces() = 0;
//
//    constexpr static uint16_t getModelHardwareId() { return -1; };
//    void speak(const char * msg) const;
//
//    static MobileRobot* buildRobot(ros::NodeHandle &nh,
//                                   uint16_t hardware_id);

};

#endif //MOBILICAN_HW_MOBILE_ROBOT_H
