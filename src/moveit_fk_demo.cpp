/***********************************************************************
# Copyright 2020 The UmbraTek Inc. All Rights Reserved.
#
# Software License Agreement (BSD License)
#
# Author: johnson huang<johnson@umbratek.com>
# 
***********************************************************************/

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
// api reference https://github.com/ros-planning/moveit/blob/master/moveit_ros/planning_interface/move_group_interface/include/moveit/move_group_interface/move_group_interface.h

int main(int argc, char **argv)
{
    ros::init(argc, argv, "utra_moveit_fk_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    arm.setGoalJointTolerance(0.001);

    arm.setMaxAccelerationScalingFactor(0.1);
    arm.setMaxVelocityScalingFactor(0.1);

    //  go to home pose
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    // go to this joints, plan first, and excute
    double targetPose_1[6] = {0, 0, 1.53, 0, 1.53, 0};
    std::vector<double> joint_group_positions_1(6);
    joint_group_positions_1[0] = targetPose_1[0];
    joint_group_positions_1[1] = targetPose_1[1];
    joint_group_positions_1[2] = targetPose_1[2];
    joint_group_positions_1[3] = targetPose_1[3];
    joint_group_positions_1[4] = targetPose_1[4];
    joint_group_positions_1[5] = targetPose_1[5];

    arm.setJointValueTarget(joint_group_positions_1);
    moveit::planning_interface::MoveGroupInterface::Plan plan_1;
    arm.plan(plan_1);
    sleep(2);
    arm.execute(plan_1);

    // go to home pose, go immediately
    double targetPose_2[6] = {0, 0, 0, 0, 0, 0};
    std::vector<double> joint_group_positions_2(6);
    joint_group_positions_2[0] = targetPose_2[0];
    joint_group_positions_2[1] = targetPose_2[1];
    joint_group_positions_2[2] = targetPose_2[2];
    joint_group_positions_2[3] = targetPose_2[3];
    joint_group_positions_2[4] = targetPose_2[4];
    joint_group_positions_2[5] = targetPose_2[5];

    arm.setJointValueTarget(joint_group_positions_2);
    moveit::planning_interface::MoveGroupInterface::Plan plan_2;
    arm.move();
    sleep(1);

    ros::shutdown(); 

    return 0;
}
