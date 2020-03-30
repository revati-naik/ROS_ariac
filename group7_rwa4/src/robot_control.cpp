//
// Created by zeid on 2/24/20.
//

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <vector>
#include <ros/ros.h>
#include <algorithm>
#include <ros/ros.h>
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/Order.h>
#include <osrf_gear/Proximity.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>


int main(int argc, char** argv) {
    //--Work with MoveIt
    ros::init(argc, argv, "ariac_example_node");
    ros::NodeHandle node("/ariac/arm1");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface::Options loadOptions("manipulator","/ariac/arm1/robot_description",node);
    moveit::planning_interface::MoveGroupInterface move_group(loadOptions);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // We can print the name of the reference frame for this robot.
    ROS_INFO_NAMED("Lecture5", "Planning frame: %s", move_group.getPlanningFrame().c_str());
    // We can also print the name of the end-effector link for this group.
    ROS_INFO_NAMED("Lecture5", "End effector link: %s", move_group.getEndEffectorLink().c_str());

    // We can plan a motion for this group to a desired pose for the
    // end-effector.
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = -0.200012;
    target_pose1.position.y = 0.583010;
    target_pose1.position.z = 0.724102;
    move_group.setPoseTarget(target_pose1);

    // Now, we call the planner to compute the plan and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group.move();
    ros::Duration(1.0).sleep();
    gripperToggle(true);

    ros::shutdown();

    return 0;
}
