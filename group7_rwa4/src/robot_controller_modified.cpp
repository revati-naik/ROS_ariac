//
// Created by zeid on 2/27/20.
//
#include "robot_controller.h"

/**
 * Constructor for the robot
 * Class attributes are initialized in the constructor init list
 * You can instantiate another robot by passing the correct parameter to the constructor
 */
RobotController::RobotController(std::string arm_id) :
    robot_controller_nh_("/ariac/"+arm_id),
    robot_controller_options(
        "manipulator", 
        "/ariac/"+arm_id+"/robot_description",
        robot_controller_nh_),
    robot_tf_buffer_ {},
    agv_tf_buffer_ {},
    robot_move_group_(robot_controller_options) 
{   
    // arm statuts publishing
    // arm_1_joint_trajectory_publisher_ = node.advertise<trajectory_msgs::JointTrajectory>(
    //   "/ariac/arm1/arm/command", 10);

    // subscribe for updating joint values change
    // arm_1_joint_state_subscriber = robot_controller_nh_.subscribe("/ariac/arm1/joint_states", 10,
    //     & RobotController::arm_1_joint_state_callback, this);

    // move group initialization
    // tf2_ros::Buffer robot_tf_buffer_;
    // tf2_ros::TransformListener robot_tf_listener_;

    tf2_ros::TransformListener robot_tf_listener_(robot_tf_buffer_);
    tf2_ros::TransformListener agv_tf_listener_(agv_tf_buffer_);

    robot_move_group_.setPlanningTime(20);
    robot_move_group_.setNumPlanningAttempts(10);
    robot_move_group_.setPlannerId("RRTConnectkConfigDefault");
    robot_move_group_.setMaxVelocityScalingFactor(0.9);
    robot_move_group_.setMaxAccelerationScalingFactor(0.9);
    // robot_move_group_.setEndEffector("moveit_ee");
    robot_move_group_.allowReplanning(true);


    //--These are joint positions used for the home position 

    // home_joint_pose_0;
    home_joint_pose_0["linear_arm_actuator_joint"] = 0;
    home_joint_pose_0["shoulder_pan_joint"] = 0;
    home_joint_pose_0["shoulder_lift_joint"] = -1.55;
    home_joint_pose_0["elbow_joint"] = 1.55;
    home_joint_pose_0["wrist_1_joint"] = 0;
    home_joint_pose_0["wrist_2_joint"] = 0;
    home_joint_pose_0["wrist_3_joint"] = 0;
    
    // home_joint_pose_1;
    home_joint_pose_1["linear_arm_actuator_joint"] = 0;
    home_joint_pose_1["shoulder_pan_joint"] = 3.14;
    home_joint_pose_1["shoulder_lift_joint"] = -2.37;
    home_joint_pose_1["elbow_joint"] = -1.64;
    home_joint_pose_1["wrist_1_joint"] = -0.69;
    home_joint_pose_1["wrist_2_joint"] = -4.7;
    home_joint_pose_1["wrist_3_joint"] = 1.01;
    
    // move_group.setJointValueTarget(home_joint_pose_0);


    SendRobotHome();


    //-- offset used for picking up parts
    //-- For the pulley_part, the offset is different since the pulley is thicker
    offset_ = 0.025;

    //--topic used to get the status of the gripper
    gripper_subscriber_ = gripper_nh_.subscribe(
            "/ariac/arm1/gripper/state", 10, &RobotController::GripperCallback, this);

    // robot_tf_listener_.waitForTransform("arm1_linear_arm_actuator", "arm1_ee_link",
    //                                         ros::Time(0), ros::Duration(10));
    // robot_tf_listener_.lookupTransform("/arm1_linear_arm_actuator", "/arm1_ee_link",
    //                                        ros::Time(0), robot_tf_transform_);

    // fixed_orientation_.x = robot_tf_transform_.getRotation().x();
    // fixed_orientation_.y = robot_tf_transform_.getRotation().y();
    // fixed_orientation_.z = robot_tf_transform_.getRotation().z();
    // fixed_orientation_.w = robot_tf_transform_.getRotation().w();


    // robot_tf_listener_(robot_tf_buffer_);
    geometry_msgs::TransformStamped transformStamped = 
        robot_tf_buffer_.lookupTransform("/arm1_linear_arm_actuator", "/arm1_ee_link", ros::Time(0));


    fixed_orientation_.x = transformStamped.transform.rotation.x;
    fixed_orientation_.y = transformStamped.transform.rotation.y;
    fixed_orientation_.z = transformStamped.transform.rotation.z;
    fixed_orientation_.w = transformStamped.transform.rotation.w;

    tf::quaternionMsgToTF(fixed_orientation_,q);
    tf::Matrix3x3(q).getRPY(roll_def_,pitch_def_,yaw_def_);


    end_position_ = home_joint_pose_1;
    // end_position_[0] = 2.2;
//    end_position_[1] = 4.5;
//    end_position_[2] = 1.2;


    // robot_tf_listener_.waitForTransform("world", "arm1_ee_link", ros::Time(0),
    //                                         ros::Duration(10));
    // robot_tf_listener_.lookupTransform("/world", "/arm1_ee_link", ros::Time(0),
    //                                        robot_tf_transform_);

    // agv_tf_listener_(agv_tf_buffer_);
    transformStamped = 
        agv_tf_buffer_.lookupTransform("/world", "/arm1_ee_link", ros::Time(0));


    home_cart_pose_.position.x = transformStamped.transform.translation.x;
    home_cart_pose_.position.y = transformStamped.transform.translation.y;
    home_cart_pose_.position.z = transformStamped.transform.translation.z;
    home_cart_pose_.orientation.x = transformStamped.transform.rotation.x;
    home_cart_pose_.orientation.y = transformStamped.transform.rotation.y;
    home_cart_pose_.orientation.z = transformStamped.transform.rotation.z;
    home_cart_pose_.orientation.w = transformStamped.transform.rotation.w;

    // agv_tf_listener_.waitForTransform("world", "kit_tray_1",
    //                                   ros::Time(0), ros::Duration(10));
    // agv_tf_listener_.lookupTransform("/world", "/kit_tray_1",
    //                                  ros::Time(0), agv_tf_transform_);


    transformStamped = 
        agv_tf_buffer_.lookupTransform("/world", "/kit_tray_1", ros::Time(0));

    agv_position_.position.x = transformStamped.transform.translation.x;
    agv_position_.position.y = transformStamped.transform.translation.y;
    agv_position_.position.z = transformStamped.transform.translation.z + 4 * offset_;

    gripper_client_ = robot_controller_nh_.serviceClient<osrf_gear::VacuumGripperControl>(
            "/ariac/arm1/gripper/control");
    counter_ = 0;
    drop_flag_ = false;
}

RobotController::~RobotController() {}

void RobotController::GripperCallback(
        const osrf_gear::VacuumGripperState::ConstPtr& grip) {
    gripper_state_ = grip->attached;
}

void RobotController::SendRobotHome() {
    robot_move_group_.setJointValueTarget(home_joint_pose_0);
    // this->execute();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.5).sleep();
    }

    robot_move_group_.setJointValueTarget(home_joint_pose_1);
    // spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.5).sleep();
    }

     ros::Duration(2.0).sleep();
}

void RobotController::SendRobotHome(std::map<std::string, double> desire_joint_states) {
    robot_move_group_.setJointValueTarget(desire_joint_states);
    // this->execute();
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1).sleep();
    }
    // ros::Duration(2.0).sleep();
}



bool RobotController::Planner() {
    ROS_INFO_STREAM("Planning started...");
    if (robot_move_group_.plan(robot_planner_) ==
        moveit::planning_interface::MoveItErrorCode::SUCCESS) {
        plan_success_ = true;
        ROS_INFO_STREAM("Planner succeeded!");
    } else {
        plan_success_ = false;
        ROS_WARN_STREAM("Planner failed!");
    }

    return plan_success_;
}


void RobotController::Execute() {
    ros::AsyncSpinner spinner(4);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.0).sleep();
    }
}

  /// Called when a new JointState message is received.
// void arm_1_joint_state_callback(
//     const sensor_msgs::JointState::ConstPtr & joint_state_msg)
// {
//     ROS_INFO_STREAM_THROTTLE(10,
//       "Joint States arm 1 (throttled to 0.1 Hz):\n" << *joint_state_msg);
//     // ROS_INFO_STREAM("Joint States:\n" << *joint_state_msg);
//     arm_1_current_joint_states_ = *joint_state_msg;
//     // if (!arm_1_has_been_zeroed_) {
//     //   arm_1_has_been_zeroed_ = true;
//     //   ROS_INFO("Sending arm to zero joint positions...");
//     //   send_arm_to_zero_state(arm_1_joint_trajectory_publisher_);
//     // }
// }

void RobotController::GoToTarget(const geometry_msgs::Pose& pose) {
    target_pose_.orientation = fixed_orientation_;
    target_pose_.position = pose.position;
    ros::AsyncSpinner spinner(4);
    robot_move_group_.setPoseTarget(target_pose_);
    spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.5).sleep();
    }
    ROS_INFO_STREAM("Point reached...");
}

void RobotController::GoToTarget(
        std::initializer_list<geometry_msgs::Pose> list) 
{
    ros::AsyncSpinner spinner(4);
    spinner.start();

    std::vector<geometry_msgs::Pose> waypoints;
    for (auto i : list) {
        i.orientation.x = fixed_orientation_.x;
        i.orientation.y = fixed_orientation_.y;
        i.orientation.z = fixed_orientation_.z;
        i.orientation.w = fixed_orientation_.w;
        waypoints.emplace_back(i);
    }

    moveit_msgs::RobotTrajectory traj;
    auto fraction =
            robot_move_group_.computeCartesianPath(waypoints, 0.01, 0.0, traj, true);

    ROS_WARN_STREAM("Fraction: " << fraction * 100);
    ros::Duration(5.0).sleep();

    robot_planner_.trajectory_ = traj;

    //if (fraction >= 0.3) {
        robot_move_group_.execute(robot_planner_);
        ros::Duration(5.0).sleep();
//    } else {
//        ROS_ERROR_STREAM("Safe Trajectory not found!");
//    }
}

void RobotController::GoDown(double z_down=0.0) {
    /*
    1. report current joint state
    2. Using the ik to figure out going down along z axis
    */

    geometry_msgs::TransformStamped transformStamped = 
        agv_tf_buffer_.lookupTransform("/world", "/arm1_ee_link", ros::Time(0));

    target_pose_.orientation = transformStamped.transform.rotation;
    target_pose_.position = transformStamped.transform.translation;

    // target_pose_.position.x = robot_tf_transform_.getOrigin().x();
    // target_pose_.position.y = robot_tf_transform_.getOrigin().y();
    // target_pose_.position.z = robot_tf_transform_.getOrigin().z();
    // target_pose_.orientation.x = robot_tf_transform_.getRotation().x();
    // target_pose_.orientation.y = robot_tf_transform_.getRotation().y();
    // target_pose_.orientation.z = robot_tf_transform_.getRotation().z();
    // target_pose_.orientation.w = robot_tf_transform_.getRotation().w();
    // target_pose_.orientation = robot_tf_transform_.getRotation();
    // target_pose_.position = robot_tf_transform_.getOrigin();
    if (z_down != 0.0)
        target_pose_.position.z -= offset_;
    else
        target_pose_.position.z -= z_down;

    // ros::AsyncSpinner spinner(4);
    robot_move_group_.setPoseTarget(target_pose_);
    // spinner.start();
    if (this->Planner()) {
        robot_move_group_.move();
        ros::Duration(1.5).sleep();
    }
    ROS_INFO_STREAM("Point reached...");
}



void RobotController::GripperToggle(const bool& state) {
    gripper_service_.request.enable = state;
    gripper_client_.call(gripper_service_);
    ros::Duration(1.0).sleep();
    // if (gripper_client_.call(gripper_service_)) {
    if (gripper_service_.response.success) {
        ROS_INFO_STREAM("Gripper activated!");
    } else {
        ROS_WARN_STREAM("Gripper activation failed!");
    }
}

// bool RobotController::dropPart(geometry_msgs::Pose part_pose) {
//   counter_++;

//   pick = false;
//   drop = true;

//   ROS_WARN_STREAM("Dropping the part number: " << counter_);

//   // ROS_INFO_STREAM("Moving to end of conveyor...");
//   // robot_move_group_.setJointValueTarget(part_pose);
//   // this->execute();
//   // ros::Duration(1.0).sleep();
//   // this->gripper_state_check(part_pose);

//   if (drop == false) {
//     // ROS_INFO_STREAM("I am stuck here..." << object);
//     ros::Duration(2.0).sleep();
//     return drop;
//   }
//   ROS_INFO_STREAM("Dropping on AGV...");

//   // agv_position_.position.x -= 0.1;
//   // if (counter_ == 1) {
//   //   agv_position_.position.y -= 0.1;
//   // }
//   // if (counter_ >= 2) {
//   //   agv_position_.position.y += 0.1;
//   //   // agv_position_.position.x +=0.1;
//   // }

//   auto temp_pose = part_pose;
//   // auto temp_pose = agv_position_;
//   temp_pose.position.z += 0.35;
//   // temp_pose.position.y += 0.5;

//   // this->setTarget(part_pose);
//   // this->execute();
//   // ros::Duration(1.0).sleep();
//   this->goToTarget({temp_pose, part_pose});
//   ros::Duration(1).sleep();
//   ROS_INFO_STREAM("Actuating the gripper...");
//   this->gripperToggle(false);

//   // ROS_INFO_STREAM("Moving to end of conveyor...");
//   // robot_move_group_.setJointValueTarget(end_position_);
//   // this->execute();
//   // ros::Duration(1.0).sleep();

//   ROS_INFO_STREAM("Going to home...");
//   // this->sendRobotHome();
//   // temp_pose = home_cart_pose_;
//   // temp_pose.position.z -= 0.05;
//   this->goToTarget({temp_pose, home_cart_pose_});
//   return drop;
// }

bool RobotController::DropPart(geometry_msgs::Pose part_pose) {
    // counter_++;

    drop_flag_ = true;

    ros::spinOnce();
    ROS_INFO_STREAM("Placing phase activated...");

    if (gripper_state_){//--while the part is still attached to the gripper
        //--move the robot to the end of the rail
         ROS_INFO_STREAM("Moving towards AGV1...");
         robot_move_group_.setJointValueTarget(end_position_);
         this->Execute();
         ros::Duration(1.0).sleep();
         ROS_INFO_STREAM("Actuating the gripper...");
         this->GripperToggle(false);
    }

    drop_flag_ = false;
    return gripper_state_;
}


bool RobotController::PickPart(geometry_msgs::Pose& part_pose) {
    // gripper_state = false;
    // pick = true;
    //ROS_INFO_STREAM("fixed_orientation_" << part_pose.orientation = fixed_orientation_);
    //ROS_WARN_STREAM("Picking the part...");

    ROS_INFO_STREAM("Moving to part...");
    part_pose.position.z = part_pose.position.z + offset_;
    auto temp_pose_1 = part_pose;
    temp_pose_1.position.z += 0.3;

    this->GoToTarget({temp_pose_1, part_pose});

    ROS_INFO_STREAM("Actuating the gripper..." << part_pose.position.z);
    this->GripperToggle(true);
    ros::spinOnce();
    while (!gripper_state_) {
        part_pose.position.z -= 0.01;
        this->GoToTarget({temp_pose_1, part_pose});
        ROS_INFO_STREAM("Actuating the gripper...");
        this->GripperToggle(true);
        ros::spinOnce();
    }

    ROS_INFO_STREAM("Going to waypoint...");
    this->GoToTarget(temp_pose_1);
    return gripper_state_;
}


bool RobotController::PickPart() {
    // gripper_state = false;
    // pick = true;
    //ROS_INFO_STREAM("fixed_orientation_" << part_pose.orientation = fixed_orientation_);
    //ROS_WARN_STREAM("Picking the part...");

    ROS_INFO_STREAM("Moving to part...");
    // part_pose.position.z = part_pose.position.z + offset_;
    // auto temp_pose_1 = part_pose;
    // temp_pose_1.position.z += 0.3;

    // this->GoToTarget({temp_pose_1, part_pose});
    this->GoDown();

    ROS_INFO_STREAM("Actuating the gripper");
    this->GripperToggle(true);
    ros::spinOnce();
    while (!gripper_state_) {
        // part_pose.position.z -= 0.01;
        // this->GoToTarget({temp_pose_1, part_pose});
        this->GoDown(0.01);
        ROS_INFO_STREAM("Actuating the gripper...");
        this->GripperToggle(true);
        ros::spinOnce();
    }

    // ROS_INFO_STREAM("Going to waypoint...");
    // this->GoToTarget(temp_pose_1);
    SendRobotHome(home_joint_pose_1);
    return gripper_state_;
}