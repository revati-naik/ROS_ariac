//
// Created by zeid on 2/27/20.
//

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <ros/ros.h>
#include <stdarg.h>
#include <tf/transform_listener.h>


#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <tf2_ros/buffer.h>


#include <iostream>
#include <string>
#include <initializer_list>

#include <osrf_gear/VacuumGripperControl.h>
#include <osrf_gear/VacuumGripperState.h>

class RobotController {
public:
    RobotController(std::string arm_id);
    ~RobotController();
    bool Planner();
    void Execute();
    void GoToTarget(std::initializer_list<geometry_msgs::Pose> list);
    void GoToTarget(const geometry_msgs::Pose& pose);
    void SendRobotHome();
    void SendRobotHome(std::map<std::string, double>);
    bool DropPart(geometry_msgs::Pose pose);
    void GripperToggle(const bool& state);
    void GripperCallback(const osrf_gear::VacuumGripperState::ConstPtr& grip);
    void GripperStateCheck(geometry_msgs::Pose pose);
    bool PickPart(geometry_msgs::Pose& part_pose);
    void GoDown(double);
    bool PickPart();

private:
    // node, publisher, subscriber
    ros::NodeHandle robot_controller_nh_;
    // ros::Publisher arm_1_joint_trajectory_publisher_;
    // ros::Subscriber arm_1_joint_state_subscriber;


    // tf
    // tf::TransformListener robot_tf_listener_;
    // tf::StampedTransform robot_tf_transform_;
    // tf::TransformListener agv_tf_listener_;
    // tf::StampedTransform agv_tf_transform_;


    tf2_ros::Buffer robot_tf_buffer_;
    // tf2_ros::TransformListener robot_tf_listener_;
    tf2_ros::Buffer agv_tf_buffer_;
    // tf2_ros::TransformListener agv_tf_listener_;


    // Moveit
    moveit::planning_interface::MoveGroupInterface::Options robot_controller_options;
    moveit::planning_interface::MoveGroupInterface robot_move_group_;
    moveit::planning_interface::MoveGroupInterface::Plan robot_planner_;

    // Gripper
    ros::ServiceClient gripper_client_;
    ros::NodeHandle gripper_nh_;
    ros::Subscriber gripper_subscriber_;
    osrf_gear::VacuumGripperControl gripper_service_;
    osrf_gear::VacuumGripperState gripper_status_;


    // position/orientation related
    std::map<std::string, double> home_joint_pose_0;
    std::map<std::string, double> home_joint_pose_1;
    // std::vector<double> home_joint_pose_;
    // std::vector<double> end_position_;
    std::map<std::string, double> end_position_;
    geometry_msgs::Pose home_cart_pose_;
    geometry_msgs::Quaternion fixed_orientation_;
    geometry_msgs::Pose agv_position_;
    geometry_msgs::Pose target_pose_;

    // Attribute and Flag
    std::string object;
    bool plan_success_;
    double offset_;
    tf::Quaternion q;
    double roll_def_,pitch_def_,yaw_def_;
    int counter_;
    bool gripper_state_, drop_flag_;
};
