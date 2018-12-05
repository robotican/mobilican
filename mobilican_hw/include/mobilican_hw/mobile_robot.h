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
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <hardware_interface/robot_hw.h>
#include "mobilican_hw/utils.h"
#include "mobilican_hw/hardware/ric_client.h"


#define RIC_DEAD_TIMEOUT            1.5 //secs

class RicClient;

class MobileRobot : public hardware_interface::RobotHW
{

protected:

    ros::NodeHandle* nh_ = nullptr;
    ros::Publisher ric_servo_pub_,
            espeak_pub_,
            diagnos_pub_;

    RicClient* ric_client_ = nullptr;

public:

    MobileRobot(ros::NodeHandle & nh, RicClient ric_client);

    void sendDiagnosticsMsg(const diagnostic_msgs::DiagnosticStatus &status) const;

    // ric observer methods
    virtual void onEncoderMsg(const ric_interface_ros::Encoder::ConstPtr& msg) = 0;
    virtual void onOrientationMsg(const ric_interface_ros::Orientation::ConstPtr& msg) = 0;
    virtual void onProximityMsg(const ric_interface_ros::Proximity::ConstPtr& msg) = 0;
    virtual void onLoggerMsg(const ric_interface_ros::Logger::ConstPtr& msg) = 0;
    virtual void onLocationMsg(const ric_interface_ros::Location::ConstPtr& msg) = 0;
    virtual void onBatteryMsg(const ric_interface_ros::Battery::ConstPtr& msg) = 0;
    virtual void onKeepAliveTimeout();

    // general robot methods
    virtual void read(const ros::Time &time, const ros::Duration& duration) = 0;
    virtual void write(const ros::Time &time, const ros::Duration& duration) = 0;

    virtual void registerInterfaces() = 0;

    constexpr static uint16_t getModelHardwareId() { return -1; };
    void speak(const char * msg) const;



};

#endif //MOBILICAN_HW_MOBILE_ROBOT_H
