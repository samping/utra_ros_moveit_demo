/***********************************************************************
# Copyright 2020 The UmbraTek Inc. All Rights Reserved.
#
# Software License Agreement (BSD License)
#
# Author: johnson huang<johnson@umbratek.com>
# 
***********************************************************************/

#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "utra_moveit_fk_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    std::string end_effector_link = arm.getEndEffectorLink();

    // set pose reference Coordinate System
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    arm.allowReplanning(true);

    // set tolerance
    arm.setGoalPositionTolerance(0.001); // meter unit
    arm.setGoalOrientationTolerance(0.01); // rad unit

    // set max velocity acceleration
    arm.setMaxAccelerationScalingFactor(0.1);
    arm.setMaxVelocityScalingFactor(0.1);

    // go to the init pose
    arm.setNamedTarget("init");
    arm.move();
    sleep(3);

    // get current pose     
    geometry_msgs::PoseStamped current_pose = arm.getCurrentPose();

    // set target pose
    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = current_pose.pose.orientation.x;
    target_pose.orientation.y = current_pose.pose.orientation.y;
    target_pose.orientation.z = current_pose.pose.orientation.z;
    target_pose.orientation.w = current_pose.pose.orientation.w;

    target_pose.position.x = current_pose.pose.position.x;
    target_pose.position.y = current_pose.pose.position.y;
    target_pose.position.z = current_pose.pose.position.z-0.1; // move down 0.1m

    arm.setStartStateToCurrentState();
    //set to target pose and move
    arm.setPoseTarget(target_pose);

    // start plan first
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::planning_interface::MoveItErrorCode success = arm.plan(plan);

    ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");   

    //execute the plan
    if(success)
      arm.execute(plan);
    sleep(1);


    ros::shutdown(); 

    return 0;
}
