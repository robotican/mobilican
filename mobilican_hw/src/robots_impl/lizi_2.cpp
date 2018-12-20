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

#include "mobilican_hw/robots_impl/lizi_2.h"


Lizi_2::Lizi_2(ros::NodeHandle &nh, RicClient& ric_client) : MobileRobot(nh, ric_client) {
    urf_rear_pub_ = nh.advertise<sensor_msgs::Range>("urf/rear", 10);
    urf_right_pub_ = nh.advertise<sensor_msgs::Range>("urf/right", 10);
    urf_left_pub_ = nh.advertise<sensor_msgs::Range>("urf/left", 10);
    imu_pub_ = nh.advertise<sensor_msgs::Imu>("imu/data", 10);
    mag_pub_ = nh.advertise<sensor_msgs::MagneticField>("imu/magnetic", 10);
    gps_pub_ = nh.advertise<sensor_msgs::NavSatFix>("gps", 10);
    battery_pub_ = nh.advertise<sensor_msgs::BatteryState>("battery", 10);

    front_right_wheel_.joint_name = "front_right_wheel_joint";
    front_left_wheel_.joint_name = "front_left_wheel_joint";
    rear_right_wheel_.joint_name = "rear_right_wheel_joint";
    rear_left_wheel_.joint_name = "rear_left_wheel_joint";

    front_right_wheel_.id = 1;
    front_left_wheel_.id = 2;
    rear_right_wheel_.id = 3;
    rear_left_wheel_.id = 4;

    if (!nh.getParam("lpf/front_right", front_right_wheel_.vel_lpf_alpha) ||
        !nh.getParam("lpf/front_left", front_left_wheel_.vel_lpf_alpha) ||
        !nh.getParam("lpf/rear_right", rear_right_wheel_.vel_lpf_alpha) ||
        !nh.getParam("lpf/rear_left", rear_left_wheel_.vel_lpf_alpha))
    {
        Utils::terminateNode("one of the lpf params is missing. "
                             "check config file. shutting down.");
    }

    if (!nh.getParam("reverse_command/front_right", front_right_wheel_.reverse_command) ||
        !nh.getParam("reverse_command/front_left", front_left_wheel_.reverse_command) ||
        !nh.getParam("reverse_command/rear_right", rear_right_wheel_.reverse_command) ||
        !nh.getParam("reverse_command/rear_left", rear_left_wheel_.reverse_command))
    {
        Utils::terminateNode("one of the reverse_command params is missing. "
                             "check config file. shutting down.");
    }

    if (!nh.getParam("reverse_feedback/front_right", front_right_wheel_.reverse_feedback) ||
        !nh.getParam("reverse_feedback/front_left", front_left_wheel_.reverse_feedback) ||
        !nh.getParam("reverse_feedback/rear_right", rear_right_wheel_.reverse_feedback) ||
        !nh.getParam("reverse_feedback/rear_left", rear_left_wheel_.reverse_feedback))
    {
        Utils::terminateNode("one of the reverse_feedback params is missing. \n"
                             "check config file. shutting down.");
    }

    std::vector<wheel*> wheels(4);
    wheels[0] = &rear_right_wheel_;
    wheels[1] = &rear_left_wheel_;
    wheels[2] = &front_right_wheel_;
    wheels[3] = &front_left_wheel_;

    wheels_control_.init(nh, wheels);

    vels_lpf_.init(nh, wheels);

    nh.getParam("ric_servo_bias", ric_servo_bias_);

    control_loop_interval_ = 0.006;
    nh.getParam("control_loop_interval", control_loop_interval_);

    // motors over voltage protection
    double protect_err_thresh = 0.9;
    double protect_time_thresh = 5.0;
    int protect_output_thresh = 200;
    bool enable_protect = true;
    nh.getParam("enable_motors_protection", enable_protect);
    nh.getParam("protection_error_threshold", protect_err_thresh);
    nh.getParam("protection_time_threshold", protect_time_thresh);
    nh.getParam("protection_output_threshold", protect_output_thresh);

    if (enable_protect) {
        wheels_control_.enableOVProtection(protect_time_thresh,
                                           protect_err_thresh,
                                           protect_output_thresh);
    } else {
        ROS_WARN("Over voltage protection disabled. "
                 "Risk of motor malfunction in case of high voltage");
    }
    vel_delta_timer_ = nh.createTimer(ros::Duration(control_loop_interval_), &Lizi_2::onControlLoopTimer, this);
}

void Lizi_2::onControlLoopTimer(const ros::TimerEvent &) {
    double delta_t = control_loop_interval_;
    for (auto &wheel : wheels_control_.getWheels()) {
        std::unique_lock<std::mutex> wheel_lock(wheel->lock);
        double delta_x = wheel->position - wheel->last_position;
        wheel->raw_velocity = delta_x / delta_t;
        wheel->last_position = wheel->position;
    }
    vels_lpf_.update();
    try{
        wheels_control_.update(ros::Duration(delta_t));
    } catch (std::runtime_error) {
        ric_client_->terminateRic();
        Utils::terminateNode("motors over voltage protection. shutting down");
    }
    for (wheel *w : wheels_control_.getWheels()) {
        if (w->reverse_command) {
            w->command_effort *= -1;
        }
        double servo_command = w->command_effort;

        // Add bias if needed. This is neccessary because lizi motors
        // only start moving after a certain command threshold. Ideally
        // the bias = |cmd_thresh - neutral_cmd|
        if (servo_command > 0) {
            servo_command += ric_servo_bias_;
        } else if (servo_command < 0) {
            servo_command -= ric_servo_bias_;
        }
        servo_command = boost::algorithm::clamp(servo_command, -500, 500);
        ric_client_->writeServoCommand((servo_command + 1500), w->id);
    }
}

void Lizi_2::registerInterfaces() {
    for (wheel *w : wheels_control_.getWheels()) {
        hardware_interface::JointStateHandle state_handle(w->joint_name,
                                                          &w->position,
                                                          &w->velocity,
                                                          &w->effort);
        joint_state_interface_.registerHandle(state_handle);
        hardware_interface::JointHandle joint_handle(joint_state_interface_.getHandle(w->joint_name),
                                                     &w->command_velocity);
        vel_joint_interface_.registerHandle(joint_handle);
    }
    registerInterface(&joint_state_interface_);
    registerInterface(&vel_joint_interface_);
}


void Lizi_2::updateWheelPosition(wheel &wheel, double new_pos) {
    std::unique_lock<std::mutex> wheel_lock(wheel.lock);
    if (wheel.reverse_feedback) {
        new_pos *= -1;
    }
    wheel.position = new_pos;
}


void Lizi_2::onEncoderMsg(const ric_interface_ros::Encoder::ConstPtr& msg)
{
    diagnostic_msgs::DiagnosticStatus diag_stat;
    diag_stat.hardware_id = std::to_string(msg->id);
    double new_pos = Utils::ticksToRads(msg->ticks, ENC_TICKS_PER_ROUND);
    if (msg->id == front_left_wheel_.id) {
        diag_stat.name = "front_left_motor";
        updateWheelPosition(front_left_wheel_, new_pos);
    } else if (msg->id == front_right_wheel_.id) {
        diag_stat.name = "front_right_motor";
        updateWheelPosition(front_right_wheel_, new_pos);
    } else if (msg->id == rear_left_wheel_.id) {
        diag_stat.name = "rear_left_motor";
        updateWheelPosition(rear_left_wheel_, new_pos);
    } else if (msg->id == rear_right_wheel_.id) {
        diag_stat.name = "rear_right_motor";
        updateWheelPosition(rear_right_wheel_, new_pos);
    }
    diagnostic_msgs::KeyValue key_val;
    key_val.key = "ticks";
    key_val.value = std::to_string(msg->ticks);
    diag_stat.values.push_back(key_val);
    if (msg->status == ric::protocol::package::Status::CRITICAL) {
        diag_stat.message = "failed to read encoder";
        diag_stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    } else if (msg->status == ric::protocol::package::Status::OK) {
        diag_stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    }
    sendDiagnosticsMsg(diag_stat);
}

void Lizi_2::onLocationMsg(const ric_interface_ros::Location::ConstPtr &msg) {
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

void Lizi_2::onBatteryMsg(const ric_interface_ros::Battery::ConstPtr &msg) {

    diagnostic_msgs::DiagnosticStatus diag_stat;
    diag_stat.name = "battery";
    diag_stat.hardware_id = std::to_string(msg->id);

    diagnostic_msgs::KeyValue key_val;
    key_val.key = "voltage";
    key_val.value = std::to_string(msg->value);
    diag_stat.values.push_back(key_val);

    if (msg->status == ric::protocol::package::Status::CRITICAL) {
        diag_stat.message = "failed to read battery";
        diag_stat.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    } else if (msg->status == ric::protocol::package::Status::WARN) {
        diag_stat.message = "low battery";
        diag_stat.level = diagnostic_msgs::DiagnosticStatus::WARN;
    } else if (msg->status == ric::protocol::package::Status::OK) {
        diag_stat.level = diagnostic_msgs::DiagnosticStatus::OK;
    }
    sensor_msgs::BatteryState batt_msg;
    batt_msg.header.frame_id = "base_link";
    batt_msg.voltage = msg->value;
    batt_msg.capacity = 11.1;
    batt_msg.percentage = ((msg->value - BATT_MIN)  / (BATT_MAX - BATT_MIN)) * 100.0;
    batt_msg.power_supply_technology = sensor_msgs::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;
    batt_msg.location = "bottom of the robot";
    for (int i=0; i<BATT_CELLS; i++) {
        batt_msg.cell_voltage.push_back(msg->value / BATT_CELLS);
    }
    battery_pub_.publish(batt_msg);
    sendDiagnosticsMsg(diag_stat);
}

void Lizi_2::onLoggerMsg(const ric_interface_ros::Logger::ConstPtr &msg) {
    switch(msg->sevirity) {
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

void Lizi_2::onOrientationMsg(const ric_interface_ros::Orientation::ConstPtr &msg) {
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
    sendDiagnosticsMsg(diag_stat);
}

void Lizi_2::onProximityMsg(const ric_interface_ros::Proximity::ConstPtr &msg) {
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
