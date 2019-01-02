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
/* Author: Elhay Rauper */

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_cartesian_path");

    ros::NodeHandle nh;
    // use async spinner when working with moveit
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // select group of joints
    moveit::planning_interface::MoveGroupInterface group("arm");

    // choose your preferred planner
    group.setPlannerId("RRTConnectkConfigDefault");

    // choose planning referecne frame
    group.setPoseReferenceFrame("base_footprint");
    ROS_INFO_STREAM("Planning to move " <<
                                        group.getEndEffectorLink() << " to a target pose expressed in " <<
                                        group.getPlanningFrame());

    // initiate robot start state
    group.setStartStateToCurrentState();

    group.setMaxVelocityScalingFactor(1.0);
    group.setNumPlanningAttempts(10);

    // provide some goal tolerance to improve success rates
    group.setGoalPositionTolerance(0.01);
    group.setGoalOrientationTolerance(0.02);
    //group_arm_torso.setGoalJointTolerance(0.01);
    //group_arm_torso.setGoalTolerance(0.05);

    //set maximum time to find a plan
    group.setPlanningTime(1.0);

    // get current end effector pose
    geometry_msgs::PoseStamped current_pose = group.getCurrentPose();

    // set current eef pose as path start pose
    geometry_msgs::Pose start_pose;
    start_pose.position.x = current_pose.pose.position.x; //0.665
    start_pose.position.y = current_pose.pose.position.y; //0.1
    start_pose.position.z = current_pose.pose.position.z; //0.6
    start_pose.orientation = current_pose.pose.orientation;

    // build horizontal square path
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(start_pose);

    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.7;
    target_pose.position.y = 0.2;
    target_pose.position.z = 0.6;
    target_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
    // 1st path pose
    waypoints.push_back(target_pose);

    // 2nd path pose: move right
    target_pose.position.y -= 0.4;
    waypoints.push_back(target_pose);

    // 3nd path pose: move backwards
    target_pose.position.x -= 0.2;
    waypoints.push_back(target_pose);

    // 4nd path pose: move left
    target_pose.position.y += 0.2;
    waypoints.push_back(target_pose);

    // we want the cartesian path to be interpolated at a resolution of 1 cm
    // which is why we will specify 0.01 as the max step in cartesian
    // translation.  We will specify the jump threshold as 0.0, effectively
    // disabling it.
    moveit_msgs::RobotTrajectory trajectory;
    double fraction = group.computeCartesianPath(waypoints,
                                                 0.01,  // eef_step
                                                 0.0,   // jump_threshold
                                                 trajectory);
    ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
             fraction * 100.0);

    // sleep to give Rviz time to visualize the plan
    sleep(5.0);

    // execute path
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    my_plan.trajectory_= trajectory;
    moveit::planning_interface::MoveItErrorCode err = group.execute(my_plan);
    if ( err == moveit::planning_interface::MoveItErrorCode::SUCCESS )
    {
        ROS_INFO("Plan found");

        // Execute the plan
        ros::Time start = ros::Time::now();
        spinner.stop();
    }
    else
        ROS_INFO("FAILED\n");

    return EXIT_SUCCESS;
}