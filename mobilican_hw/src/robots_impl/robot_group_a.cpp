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


#include "mobilican_hw/robots_impl/robot_group_a.h"

RobotGroupA::RobotGroupA(ros::NodeHandle &nh, RicClient &ric_client) :
        MobileRobot(nh, ric_client){

    urf_rear_pub_ = nh.advertise<sensor_msgs::Range>("urf/rear", 10);
    urf_right_pub_ = nh.advertise<sensor_msgs::Range>("urf/right", 10);
    urf_left_pub_ = nh.advertise<sensor_msgs::Range>("urf/left", 10);
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data", 10);
    mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("imu/magnetic", 10);
    gps_pub_ = nh.advertise<sensor_msgs::NavSatFix>("gps", 10);
}


void RobotGroupA::onLocationMsg(const ric_interface_ros::Location::ConstPtr &msg) {
    diagnostic_msgs::DiagnosticStatus diag_stat;
    diag_stat.name = "gps";
    diag_stat.hardware_id = std::to_string(msg->id);

    sensor_msgs::NavSatFix gps_msg;
    gps_msg.header.frame_id = "base_link";
    gps_msg.latitude = msg->lat;
    gps_msg.longitude = msg->lon;
    gps_msg.altitude = msg->alt;
    gps_msg.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;

    if (msg->status == ric::protocol::package::Status::CRITICAL) {
        diag_stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
        diag_stat.message = "failed to read GPS";
    } else if (msg->status == ric::protocol::package::Status::WARN) {
        gps_msg.status.status = sensor_msgs::NavSatStatus::STATUS_NO_FIX;
        gps_pub_.publish(gps_msg);

        diag_stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
        diag_stat.message = "no fix";
    } else if (msg->status == ric::protocol::package::Status::OK) {
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


void RobotGroupA::onOrientationMsg(const ric_interface_ros::Orientation::ConstPtr &msg) {
    diagnostic_msgs::DiagnosticStatus diag_stat;
    diag_stat.name = "imu";
    diag_stat.hardware_id = std::to_string(msg->id);

    if (msg->status == ric::protocol::package::Status::CRITICAL) {
        diag_stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
        diag_stat.message = "failed to initialize and read from IMU";
    } else if (msg->status == ric::protocol::package::Status::OK) {
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
    if (ros::Time::now() - orientation_stat_time_ > ros::Duration(1)) {
        sendDiagnosticsMsg(diag_stat);
        orientation_stat_time_ = ros::Time::now();
    }
}

void RobotGroupA::onProximityMsg(const ric_interface_ros::Proximity::ConstPtr &msg) {
    sensor_msgs::Range range_msg;
    range_msg.header.stamp = ros::Time::now();
    range_msg.min_range = 0.3;
    range_msg.max_range = 3.0;
    range_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    range_msg.range = msg->distance / 1000.0;
    range_msg.field_of_view = 0.7f;

    int urf_id = msg->id;

    switch (urf_id) {
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