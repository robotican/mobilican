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

#include "mobilican_hw/robots_impl/komodo_2.h"

/*
 * Komodo have only 2 motors and encoders, but because it have
 * 4 wheels, we have to simulate the front wheels so ROS can
 * use them currectly in joint_states
 */

Komodo_2::Komodo_2(ros::NodeHandle & nh, RicClient & ric_client) :
                        MobileRobot(nh, ric_client) , roboteq_(nh), bms_(nh)
{
    urf_rear_pub_ = nh.advertise<sensor_msgs::Range>("urf/rear", 10);
    urf_right_pub_ = nh.advertise<sensor_msgs::Range>("urf/right", 10);
    urf_left_pub_ = nh.advertise<sensor_msgs::Range>("urf/left", 10);
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data", 10);
    mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("imu/magnetic", 10);
    gps_pub_ = nh.advertise<sensor_msgs::NavSatFix>("gps", 10);

    bms_.connect("/dev/mobilican/BMS");

    // try connect to roboteq, and send real wheels to it
    std::vector<std::string> real_wheels(2);
    real_wheels.emplace_back("right_rear_wheel_joint");
    real_wheels.emplace_back("left_rear_wheel_joint");
    roboteq_.connect("/dev/mobilican/ROBOTEQ", 115200, real_wheels);
    roboteq_.getMotors(motors_);

    virtual_wheels_[0].joint_name = "right_front_wheel_joint";
    virtual_wheels_[1].joint_name = "left_front_wheel_joint";
}

void Komodo_2::registerInterfaces()
{
    // register real motors and encoders (represented as rear wheels)
    for (roboteq::Motor * m : *motors_)
    {
        joint_state_interface_.registerHandle(m->joint_state_handle);
        vel_joint_interface_.registerHandle(m->joint_handle);
    }

    // register virtual wheels (represented as front wheels)
    for (int i=0; i<2; i++)
    {
        hardware_interface::JointStateHandle state_handle(virtual_wheels_[i].joint_name,
                                                          &virtual_wheels_[i].position,
                                                          &virtual_wheels_[i].velocity,
                                                          &virtual_wheels_[i].effort);
        joint_state_interface_.registerHandle(state_handle);
        hardware_interface::JointHandle joint_handle(joint_state_interface_.getHandle(virtual_wheels_[i].joint_name),
                                                     &virtual_wheels_[i].command_velocity);
        vel_joint_interface_.registerHandle(joint_handle);
    }

    registerInterface(&joint_state_interface_);
    registerInterface(&vel_joint_interface_);
}

void Komodo_2::write(const ros::Time &time, const ros::Duration &duration)
{
    roboteq_.write(time, duration);
}

void Komodo_2::read(const ros::Time &time, const ros::Duration &duration)
{
    roboteq_.read(time, duration);

    // simulate front virtual wheels be matching
    // their values to real rear wheels values
    for (int i=0; i<2; i++)
    {
        virtual_wheels_[i].position = (*motors_)[i]->position;
        virtual_wheels_[i].velocity = (*motors_)[i]->velocity;
        virtual_wheels_[i].effort = (*motors_)[i]->effort;
    }
}

void Komodo_2::onLocationMsg(const ric_interface_ros::Location::ConstPtr &msg)
{
    diagnostic_msgs::DiagnosticStatus diag_stat;
    diag_stat.name = "gps";
    diag_stat.hardware_id = std::to_string(msg->id);

    sensor_msgs::NavSatFix gps_msg;
    gps_msg.header.frame_id = "base_link";
    gps_msg.latitude = msg->lat;
    gps_msg.longitude = msg->lon;
    gps_msg.altitude = msg->alt;
    gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

    if (msg->status == ric::protocol::package::Status::READ_FAIL)
    {
        diag_stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
        diag_stat.message = "failed to read GPS";
    }
    else if (msg->status == ric::protocol::package::Status::READ_WARN)
    {
        gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        gps_pub_.publish(gps_msg);

        diag_stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
        diag_stat.message = "no fix";
    }
    else if (msg->status == ric::protocol::package::Status::OK)
    {
        sensor_msgs::NavSatFix gps_msg;
        gps_msg.header.frame_id = "base_link";
        gps_msg.latitude = msg->lat;
        gps_msg.longitude = msg->lon;
        gps_msg.altitude = msg->alt;
        gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        // if valid(OK) message arrived from ric, it must have valid fix
        gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;

        gps_pub_.publish(gps_msg);

        diag_stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    }

    sendDiagnosticsMsg(diag_stat);
}

void Komodo_2::onLoggerMsg(const ric_interface_ros::Logger::ConstPtr &msg)
{
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

void Komodo_2::onOrientationMsg(const ric_interface_ros::Orientation::ConstPtr &msg)
{
    diagnostic_msgs::DiagnosticStatus diag_stat;
    diag_stat.name = "imu";
    diag_stat.hardware_id = std::to_string(msg->id);

    if (msg->status == ric::protocol::package::Status::INIT_FAIL)
    {
        diag_stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
        diag_stat.message = "failed to initialize and read from IMU";
    }
    else if (msg->status == ric::protocol::package::Status::OK)
    {
        /* publish imu */
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "base_footprint";

        double roll, pitch, yaw;
        pitch = -msg->roll;
        roll = -msg->pitch;
        yaw = msg->yaw - M_PI / 2;

        //wrap to PI
        if (yaw > M_PI )
            yaw -= 2 * M_PI;
        else if (yaw < -M_PI)
            yaw += 2 * M_PI;

        tf::Quaternion orientation_q =
                tf::createQuaternionFromRPY(roll, pitch, yaw);

        imu_msg.orientation.x = orientation_q.x();
        imu_msg.orientation.y = orientation_q.y();
        imu_msg.orientation.z = orientation_q.z();
        imu_msg.orientation.w = orientation_q.w();
        imu_msg.angular_velocity.x = -1 * msg->gyro_y;
        imu_msg.angular_velocity.y = -1 * msg->gyro_x;
        imu_msg.angular_velocity.z = -1 * msg->gyro_z;
        imu_msg.linear_acceleration.x = msg->accl_x * Utils::G_FORCE + 0.23;
        imu_msg.linear_acceleration.y = msg->accl_y * Utils::G_FORCE - 0.13;
        imu_msg.linear_acceleration.z = msg->accl_z * Utils::G_FORCE - 0.1;
        imu_pub_.publish(imu_msg);

        sensor_msgs::MagneticField mag_msg;
        mag_msg.header.stamp = ros::Time::now();
        mag_msg.header.frame_id = "base_link";
        mag_msg.magnetic_field.x = msg->mag_x;
        mag_msg.magnetic_field.y = msg->mag_y;
        mag_msg.magnetic_field.z = msg->mag_z;
        mag_pub_.publish(mag_msg);

        diag_stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    }

    sendDiagnosticsMsg(diag_stat);
}

void Komodo_2::onProximityMsg(const ric_interface_ros::Proximity::ConstPtr &msg)
{
    sensor_msgs::Range range_msg;
    range_msg.header.stamp = ros::Time::now();
    range_msg.header.stamp = ros::Time::now();
    range_msg.min_range = 0.3;
    range_msg.max_range = 3.0;
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.range = msg->distance / 1000.0;
    range_msg.field_of_view = 0.7f;

    int urf_id = msg->id;

    switch (urf_id)
    {
        case UrfId::REAR:
            range_msg.header.frame_id = "rear_urf_link";
            urf_rear_pub_.publish(range_msg);
            break;

        case UrfId::RIGHT:
            range_msg.header.frame_id = "right_urf_link";
            urf_right_pub_.publish(range_msg);
            break;

        case UrfId::LEFT:
            range_msg.header.frame_id = "left_urf_link";
            urf_left_pub_.publish(range_msg);
            break;
    }
}