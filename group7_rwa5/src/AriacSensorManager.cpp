#include "AriacSensorManager.h"
#include <osrf_gear/AGVControl.h>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>


using namespace std;

AriacSensorManager::AriacSensorManager() :
        task {},
        arm1 {"arm1"},
        arm2 {"arm2"},
        tfListener(tfBuffer)
{
    orders_sub = sensor_nh_.subscribe("/ariac/orders", 10,
                                      & AriacSensorManager::order_callback, this);

    lc_bin_1_sub = sensor_nh_.subscribe("/ariac/lc_bin_1", 10,
                                        & AriacSensorManager::lc_bin_1_callback, this);

    lc_bin_2_sub = sensor_nh_.subscribe("/ariac/lc_bin_2", 10,
                                        &AriacSensorManager::lc_bin_2_callback, this);
    lc_bin_3_sub = sensor_nh_.subscribe("/ariac/lc_bin_3", 10,
                                        &AriacSensorManager::lc_bin_3_callback, this);

    lc_bin_4_sub = sensor_nh_.subscribe("/ariac/lc_bin_4", 10,
                                        &AriacSensorManager::lc_bin_4_callback, this);

    lc_bin_5_sub = sensor_nh_.subscribe("/ariac/lc_bin_5", 10,
                                        &AriacSensorManager::lc_bin_5_callback, this);

    lc_bin_6_sub = sensor_nh_.subscribe("/ariac/lc_bin_6", 10,
                                        &AriacSensorManager::lc_bin_6_callback, this);

    qc_1_sub = sensor_nh_.subscribe("/ariac/quality_control_sensor_1", 10,
                                    & AriacSensorManager::qc_1_callback, this);

    qc_2_sub = sensor_nh_.subscribe("/ariac/quality_control_sensor_2", 10,
                                    & AriacSensorManager::qc_2_callback, this);

    init_ = false;
    cam_1_ = false;
    cam_2_ = false;
    cam_3_ = false;
    cam_4_ = false;
    cam_5_ = false;
    cam_6_ = false;
    Flag_updateKit = false;

    NumPartsToRemove = 0;
    NumPartsToModify = 0;
    NumPartsToAdd = 0;

    camera1_frame_counter_ = 1;
    camera2_frame_counter_ = 1;
    camera3_frame_counter_ = 1;
    camera4_frame_counter_ = 1;
    camera5_frame_counter_ = 1;
    camera6_frame_counter_ = 1;

    order_counter = 0;

    order_number = 0;
    int counter = 0;

    qc_1_redFlag = false;
    qc_2_redFlag = false;
    order_receiving_flag = false;
    arm1_busy = false;
    everything_ready = false;

    arm2_check_qc_pose["linear_arm_actuator_joint"] = -0.87;
    arm2_check_qc_pose["shoulder_pan_joint"] = 4.6;
    arm2_check_qc_pose["elbow_joint"] = 0;
    arm2_check_qc_pose["shoulder_lift_joint"] = 0;
    arm2_check_qc_pose["wrist_1_joint"] = -1.57;
    arm2_check_qc_pose["wrist_2_joint"] = -3.14/2;

    arm2_transition_pose["linear_arm_actuator_joint"] = 0;
    arm2_transition_pose["shoulder_pan_joint"] = 3.14;
    arm2_transition_pose["shoulder_lift_joint"] = -1.2;
    arm2_transition_pose["elbow_joint"] = 2.01;
    arm2_transition_pose["wrist_1_joint"] = -2.51;
    arm2_transition_pose["wrist_2_joint"] = -1.51;
    arm2_transition_pose["wrist_3_joint"] = 0;

    arm1_bin_pose["linear_arm_actuator_joint"] = 0.25;
    arm1_bin_pose["shoulder_pan_joint"] = 2.13;
    arm1_bin_pose["shoulder_lift_joint"] = -0.55;
    arm1_bin_pose["elbow_joint"] = 1.1;
    arm1_bin_pose["wrist_1_joint"] = 4.15;
    arm1_bin_pose["wrist_2_joint"] = -1.57;

    arm1_check_qc_pose["linear_arm_actuator_joint"] = 1.18;
    arm1_check_qc_pose["shoulder_pan_joint"] = 1.44;
    arm1_check_qc_pose["elbow_joint"] = 1.28;
    arm1_check_qc_pose["shoulder_lift_joint"] = -0.5;
    arm1_check_qc_pose["wrist_1_joint"] = 3.94;
    arm1_check_qc_pose["wrist_2_joint"] = -1.57;

    // arm1_trans_pose["linear_arm_actuator_joint"] = -0.56;
    // arm1_trans_pose["shoulder_pan_joint"] = 3.14;
    // arm1_trans_pose["shoulder_lift_joint"] = -3.14/2;
    // arm1_trans_pose["elbow_joint"] = 3.14/2;
    // arm1_trans_pose["wrist_1_joint"] = -1.5;
    // arm1_trans_pose["wrist_2_joint"] = -3.14/2;

}

AriacSensorManager::~AriacSensorManager() {}

void AriacSensorManager::order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ROS_INFO_STREAM("[AriacSensorManager][order_callback]:Received order:\n" << *order_msg);
    received_orders_.push_back(*order_msg);
    // setDesiredParts();
    order_receiving_flag = true;
    order_counter += 1;

    if (order_counter >1) {
        Flag_updateKit = true;
        ROS_INFO_STREAM("[AriacSensorManager][order_callback]:Flag Update Set to true:\n" << Flag_updateKit);

    }else{
        ExecuteOrder();
    }
}

void AriacSensorManager::buildUpdatedKitMap(){
    ROS_INFO_STREAM("[buildUpdatedKitMap]:updated desired parts");
    auto updated_order = received_orders_.back();
    auto order_id = updated_order.order_id;
    auto shipments = updated_order.shipments;
    for (const auto &shipment: shipments){
        auto shipment_type = shipment.shipment_type;
        auto products = shipment.products;
        ROS_INFO_STREAM("Order ID: " << order_id);
        ROS_INFO_STREAM("Shipment Type: " << shipment_type);
        for (const auto &product: products){
            order_update_product_type_pose_[product.type].emplace_back(product.pose);;
        }
    }

    ROS_INFO_STREAM("[buildUpdatedKitMap]:The updated desired_parts are:");

    auto vect = order_update_product_type_pose_["disk_part"];
    for (auto part : vect){
        std::cout  << " disk_part Part Pose is: " << part << std::endl;
    }

    vect = order_update_product_type_pose_["piston_rod_part"];
    for (auto part : vect){
        std::cout  << " piston_rod_part Part Pose is: " << part << std::endl;
    }

    vect = order_update_product_type_pose_["gear_part"];
    for (auto part : vect){
        std::cout  << " gear_part Part Pose is: " << part << std::endl;
    }
}

void AriacSensorManager::lc_bin_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
//    ros::AsyncSpinner spinner(0);
//    spinner.start();

    if (init_|| cam_1_ == true) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 1: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_ERROR_STREAM("Logical Camera 1 does not see anything");

    current_parts_1_ = *image_msg;
    this->BuildProductFrames(1);
}

void AriacSensorManager::lc_bin_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
//    ros::AsyncSpinner spinner(0);
//    spinner.start();

    if (init_ || cam_2_ == true) return;

    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 2: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_ERROR_STREAM("Logical Camera 2 does not see anything");

    current_parts_2_ = *image_msg;
    this->BuildProductFrames(2);
}

void AriacSensorManager::lc_bin_3_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
//    ros::AsyncSpinner spinner(0);
//    spinner.start();

    if (init_ || cam_3_ == true) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 3: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_ERROR_STREAM("Logical Camera 3 does not see anything");

    current_parts_3_ = *image_msg;
    this->BuildProductFrames(3);
}

void AriacSensorManager::lc_bin_4_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
//    ros::AsyncSpinner spinner(0);
//    spinner.start();

    if (init_ || cam_4_ == true) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 4: '" << image_msg->models.size() << "' objects.");

    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 4: message " << *image_msg << "' objects.");

    if (image_msg->models.size() == 0)
        ROS_ERROR_STREAM("Logical Camera 4 does not see anything");



    current_parts_4_ = *image_msg;
    this->BuildProductFrames(4);
}

void AriacSensorManager::lc_bin_5_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
//    ros::AsyncSpinner spinner(0);
//    spinner.start();

    if (init_ || cam_5_ == true) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 5: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_ERROR_STREAM("Logical Camera 5 does not see anything");

    current_parts_5_ = *image_msg;
    this->BuildProductFrames(5);
}

void AriacSensorManager::lc_bin_6_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
//    ros::AsyncSpinner spinner(0);
//    spinner.start();

    if (init_ || cam_6_ == true) return;
    ROS_INFO_STREAM_THROTTLE(10,
                             "Logical camera 6: '" << image_msg->models.size() << "' objects.");
    if (image_msg->models.size() == 0)
        ROS_ERROR_STREAM("Logical Camera 6 does not see anything");

    current_parts_6_ = *image_msg;
    this->BuildProductFrames(6);
}

void AriacSensorManager::BuildProductFrames(int camera_id){

//    ros::AsyncSpinner spinner(0);
//    spinner.start();

    if (camera_id == 1) {
        for (auto& msg : current_parts_1_.models) {
            //--build the frame for each product
            std::string product_frame = "lc_bin_1_" + msg.type + "_" +
                                        std::to_string(camera1_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            camera1_frame_counter_++;
        }
        cam_1_ = true;
    }
    else if (camera_id == 2) {
        for (auto& msg : current_parts_2_.models) {
            //--build the frame for each product
            std::string product_frame = "lc_bin_2_" + msg.type + "_" +
                                        std::to_string(camera2_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            camera2_frame_counter_++;
        }
        cam_2_ = true;
    }
    else if (camera_id == 3) {
        for (auto& msg : current_parts_3_.models) {
            //--build the frame for each product
            std::string product_frame = "lc_bin_3_" + msg.type + "_" +
                                        std::to_string(camera3_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            camera3_frame_counter_++;
        }
        cam_3_ = true;
    }
    else if (camera_id == 4) {
        for (auto& msg : current_parts_4_.models) {
            //--build the frame for each product
            std::string product_frame = "lc_bin_4_" + msg.type + "_" +
                                        std::to_string(camera4_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            camera4_frame_counter_++;
        }
        cam_4_ = true;
    }
    else if (camera_id == 5) {
        for (auto& msg : current_parts_5_.models) {
            //--build the frame for each product
            std::string product_frame = "lc_bin_5_" + msg.type + "_" +
                                        std::to_string(camera5_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            camera5_frame_counter_++;
        }
        cam_5_ = true;
    }
    else if (camera_id == 6) {
        for (auto& msg : current_parts_6_.models) {
            //--build the frame for each product
            std::string product_frame = "lc_bin_6_" + msg.type + "_" +
                                        std::to_string(camera6_frame_counter_) + "_frame";

            product_frame_list_[msg.type].emplace_back(product_frame);
            camera6_frame_counter_++;
        }
        cam_6_ = true;
    }

    if (cam_1_ && cam_2_ && cam_3_ && cam_4_ && cam_5_ && cam_6_) {
        init_ = true;

        auto vect = product_frame_list_["pulley_part"];

        for (auto part : vect){
            std::cout  << " Part frame is: " << part << std::endl;
//            std::cout << "Pose:\n";
//            ROS_INFO_STREAM(part.second);
        }

        vect = product_frame_list_["disk_part"];

        for (auto part : vect){
            std::cout  << " Part frame is: " << part << std::endl;
//            std::cout << "Pose:\n";
//            ROS_INFO_STREAM(part.second);
        }

        vect = product_frame_list_["piston_rod_part"];

        for (auto part : vect){
            std::cout  << " Part frame is: " << part << std::endl;
//            std::cout << "Pose:\n";
//            ROS_INFO_STREAM(part.second);
        }


        vect = product_frame_list_["gear_part"];

        for (auto part : vect){
            std::cout  << " Part frame is: " << part << std::endl;
        }

    }
}

geometry_msgs::Pose AriacSensorManager::GetPartPose(const std::string& src_frame,
                                                    const std::string& target_frame) {
    geometry_msgs::Pose part_pose;

    ROS_INFO_STREAM("Getting part pose...");

    if (init_) {
        camera_tf_listener_.waitForTransform(src_frame, target_frame, ros::Time(0),
                                             ros::Duration(3));
        camera_tf_listener_.lookupTransform(src_frame, target_frame, ros::Time(0),
                                            camera_tf_transform_);

        part_pose.position.x = camera_tf_transform_.getOrigin().x();
        part_pose.position.y = camera_tf_transform_.getOrigin().y();
        part_pose.position.z = camera_tf_transform_.getOrigin().z();

    } else {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        this->BuildProductFrames(1);
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        this->BuildProductFrames(2);
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        this->BuildProductFrames(3);
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        this->BuildProductFrames(4);
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        this->BuildProductFrames(5);
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        this->BuildProductFrames(6);

        part_pose = this->GetPartPose(src_frame, target_frame);
    }

    return part_pose;
}

void AriacSensorManager::setDesiredParts(){
    ROS_INFO_STREAM("[setDesiredParts]:Setting desired parts");
    auto current_order = received_orders_[order_number];
    auto order_id = current_order.order_id;
    auto shipments = current_order.shipments;
    for (const auto &shipment: shipments){
        auto shipment_type = shipment.shipment_type;
        auto products = shipment.products;
        ROS_INFO_STREAM("Order ID: " << order_id);
        ROS_INFO_STREAM("Shipment Type: " << shipment_type);
        for (const auto &product: products){
            desired_parts_info.insert({product.type, product.pose});
            parts_to_pickup_belt.insert(product.type);
            ++(task[product.type]); // Adding product name in task variable
        }
    }

    ROS_INFO_STREAM("[setDesiredParts]:The current desired_parts are:");
    for (const auto & part : desired_parts_info){
        std::cout << part.first << std::endl;
        std::cout << "Pose:\n";
        ROS_INFO_STREAM(part.second);
    }

    ROS_INFO_STREAM("[ASM]:[setDesiredParts]:The Parts_to_pickup_belt set is:");
    for (auto it = parts_to_pickup_belt.begin(); it != parts_to_pickup_belt.end(); ++it)
        std::cout << *it << "\n";

    if (!order_id.empty())
        ++order_number;

    everything_ready = true;

    if (order_number <=1){
        ExecuteOrder();
    }

}

std::string AriacSensorManager::GetProductFrame(std::string product_type) {
    //--Grab the last one from the list then remove it
    if (!product_frame_list_.empty()) {
        std::string frame = product_frame_list_[product_type].back();
        ROS_INFO_STREAM("Frame >>>> " << frame);
        product_frame_list_[product_type].pop_back();
        return frame;
    } else {
        ROS_ERROR_STREAM("No product frame found for " << product_type);
        ros::shutdown();
    }
}

geometry_msgs::Pose AriacSensorManager::kitToWorld(geometry_msgs::Pose part_pose, int agv_id){

    std::string tray_ID = "kit_tray_" + std::to_string(agv_id);

    geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
    StampedPose_in.header.frame_id = tray_ID;
    StampedPose_in.pose = part_pose;


    StampedPose_out = tfBuffer.transform(StampedPose_in, "world");
    geometry_msgs::Pose part_pose_kit = StampedPose_out.pose;

    return part_pose_kit;
}

bool AriacSensorManager::QualityCheck(int agv_id){

//    ROS_INFO_STREAM("[AriacSensorManager]:[QualityCheck]: In side QualityCheck : ");

    bool quality_flag = false;
    if (agv_id == 1){
        quality_flag =  qc_1_redFlag;
    }
    else if (agv_id == 2){
        quality_flag =  qc_2_redFlag;
    }
    return quality_flag;
}

void AriacSensorManager::PickAndThrow(geometry_msgs::Pose part_pose, std::string product_type, RobotController& arm){

    ROS_INFO_STREAM("[AriacSensorManager]:[PickAndThrow]:Inside PickAndThrow ...");

    if (product_type == "pulley_part")
        part_pose.position.z += 0.08;
    //--task the robot to pick up this part
    bool failed_pick = arm.PickPart(part_pose);
    ROS_WARN_STREAM("Picking up state " << failed_pick);
    ros::Duration(0.5).sleep();

    while (!failed_pick) {
//        auto part_pose = GetPartPose("/world", product_frame);
        failed_pick = arm.PickPart(part_pose);
    }

    geometry_msgs::Pose throw_away_pose;

    throw_away_pose.position.x = 0.2-0.4;
    throw_away_pose.position.y = -3.35 + 0.5;
    throw_away_pose.position.z = 0.85 + 0.2;
    throw_away_pose.orientation.x = 0;
    throw_away_pose.orientation.y = 0;
    throw_away_pose.orientation.z = 0;
    throw_away_pose.orientation.w = 1;

    arm.DropPart(throw_away_pose);
    ros::Duration(1).sleep();

//    ros::Duration(0.5).sleep();
//
//    arm.SendRobotTo("shoulder_lift_joint", -0.25);
//    ros::Duration(0.5).sleep();
//
//    arm.SendRobotTo("shoulder_pan_joint", 4.13);
//    ros::Duration(0.5).sleep();
//
//    arm.GripperToggle(false);
//    ros::Duration(0.5).sleep();
//
//    arm.SendRobotTo("shoulder_pan_joint", 4.51);

}


void AriacSensorManager::CorrectPose(geometry_msgs::Pose current_pose, geometry_msgs::Pose updated_pose,
        std::string product_type, int agv_id){

    agv_id = 2;

    ROS_INFO_STREAM("[AriacSensorManager]:[CorrectPose]:Inside PickAndPlace ...");

    geometry_msgs::Pose part_pose = kitToWorld(current_pose, agv_id);

    if (product_type == "pulley_part")
        part_pose.position.z += 0.08;
    //--task the robot to pick up this part
    bool failed_pick = arm2.PickPart(part_pose);
//        ROS_WARN_STREAM("Picking up state " << failed_pick);
    ros::Duration(0.5).sleep();

    while (!failed_pick) {
//        auto part_pose = GetPartPose("/world", product_frame);
        failed_pick = arm2.PickPart(part_pose);
    }

    ros::Duration(0.5).sleep();
    arm2.SendRobotTo(arm2_transition_pose);
    ros::Duration(1).sleep();

    // Function to get the drop pose in world coordinates
    geometry_msgs::Pose drop_pose = kitToWorld(updated_pose, agv_id);
    ROS_INFO_STREAM("[AriacSensorManager]:[CorrectPose]: Drop Pose : " << drop_pose);

    drop_pose.position.z += 0.05;
    auto result = arm2.DropPart(drop_pose);
    ros::Duration(1.5).sleep();
    arm2.SendRobotTo(arm2_transition_pose);
    ros::Duration(1.5).sleep();
}

bool AriacSensorManager::PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id){

//    ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlace]:Inside PickAndPlace ...");

    if(init_){

        std::string product_type = product_type_pose.first;

        ROS_WARN_STREAM("[AriacSensorManager]:[PickAndPlace]:Product type >>>> " << product_type);
        std::string product_frame = this->GetProductFrame(product_type);

        ROS_WARN_STREAM("Product frame >>>> " << product_frame);
        auto part_pose = GetPartPose("/world", product_frame);

        if (product_type == "pulley_part")
            part_pose.position.z += 0.08;
        //--task the robot to pick up this part
        bool failed_pick = arm2.PickPart(part_pose);
//        ROS_WARN_STREAM("Picking up state " << failed_pick);
        ros::Duration(0.5).sleep();

        while (!failed_pick) {
            auto part_pose = GetPartPose("/world", product_frame);
            failed_pick = arm2.PickPart(part_pose);
        }

        ros::Duration(0.5).sleep();
        arm2.SendRobotTo(arm2_transition_pose);
        ros::Duration(1).sleep();

        agv_id = 2;

        // Function to get the drop pose in world coordinates
        geometry_msgs::Pose drop_pose = kitToWorld(product_type_pose.second, agv_id);
        ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlace]: Drop Pose : " << drop_pose);

        drop_pose.position.z += 0.05;
        auto result = arm2.DropPart(drop_pose);
        ros::Duration(1.5).sleep();
        arm2.SendRobotTo(arm2_transition_pose);
        ros::Duration(1.5).sleep();

        // Here we can do the quality check, return type would be a bool, whether good or bad
        bool quality = QualityCheck(agv_id);

        ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlace]: Quality is : " << quality);

        // if part is faluty:
        if (quality){

            ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlace]: In side Quality If : " << quality);

//            drop_pose.position.z -= 0.02;
            //------pick the part and throw it away using the drop_pose-------
            PickAndThrow(drop_pose, product_type, arm2);

            arm2.SendRobotTo(arm2_check_qc_pose);
            ros::Duration(1.5).sleep();

            //------ Do PickAndPlace again------------------------------------
            PickAndPlace(product_type_pose, agv_id);
        }
    }
}

void AriacSensorManager::ExecuteOrder() {
    ROS_WARN(">>>>>> Executing order...");
    //scanned_objects_ = camera_.GetParts();

    //-- used to check if pick and place was successful
    bool pick_n_place_success{false};

    std::list<std::pair<std::string,geometry_msgs::Pose>> failed_parts;

    ros::spinOnce();
    ros::Duration(1.0).sleep();
//    product_frame_list_ = camera_.get_product_frame_list();

    for (const auto &order:received_orders_){
        auto order_id = order.order_id;
        auto shipments = order.shipments;
        for (const auto &shipment: shipments){
            auto shipment_type = shipment.shipment_type;
            auto agv = shipment.agv_id.back();//--this returns a char
            //-- if agv is any then we use AGV1, else we convert agv id to int
            //--agv-'0' converts '1' to 1 and '2' to 2
            int agv_id = (shipment.agv_id == "any") ? 1 : agv - '0';

            auto products = shipment.products;
            ROS_INFO_STREAM("Order ID: " << order_id);
            ROS_INFO_STREAM("Shipment Type: " << shipment_type);
            ROS_INFO_STREAM("AGV ID: " << agv_id);
            for (const auto &product: products){
                ros::spinOnce();
                product_type_pose_.first = product.type;
                //ROS_INFO_STREAM("Product type: " << product_type_pose_.first);
                product_type_pose_.second = product.pose;
                ROS_INFO_STREAM("Product pose: " << product_type_pose_.second.position.x);
                pick_n_place_success =  PickAndPlace(product_type_pose_, agv_id);

                // We put the parts which are good into  a map here
                built_kit_product_type_pose_[product.type].emplace_back(product.pose);

                //--todo: What do we do if pick and place fails?
            }
            if (Flag_updateKit){
                buildUpdatedKitMap();
                UpdateKit();
            }
            SubmitAGV(2);
            ROS_INFO_STREAM("[AriacSensorManager][ExecuteOrder] : Submitting AGV 1");
            int finish=1;

        }
    }
}

void  AriacSensorManager::WhatToRemove(){

    vector<std::string> keys;

    // Looping through the maps
    for(auto const& item: built_kit_product_type_pose_){

        keys.push_back(item.first);
        auto part_type = item.first;

        std::cout << "[AriacSensorManager][WhatToRemove] : The part type is : " << part_type << "\n";

        // condition to check if that type of part_type exists in both the maps
        if (order_update_product_type_pose_.find(part_type) != order_update_product_type_pose_.end()){

            auto update = order_update_product_type_pose_[part_type];
            auto built = built_kit_product_type_pose_[part_type];

            if (built.size() > update.size()){
                int difference = built.size() - update.size();
                NumPartsToRemove = NumPartsToRemove + difference;  // Lets say
                std::cout << "[AriacSensorManager][WhatToRemove] : The difference is : " << difference << "\n";
                int i=0;
                bool same_flag = false;

                // Select a pose from built
                for (auto built_pose: built){
                    same_flag = false;

                    // search with every pose in update
                    for(auto update_pose: update){

                        // if the pose matches with one of the pose, dont remove
                        if (built_pose == update_pose){
                            same_flag = true;
                        }
                    }
                    // find out what to remove
                    // if the pose doesn't match with any pose in the update, we can safely remove
                    if (!same_flag && i< difference){
                        parts_to_remove_product_type_pose_[part_type].emplace_back(built_pose);

                        // Erase built_pose that will be thrown away. Using the std::remove to find the part
                        // from the vector to remove it for built_map
                        built_kit_product_type_pose_[part_type].erase(std::remove(built_kit_product_type_pose_[part_type].begin(),
                                built_kit_product_type_pose_[part_type].end(), built_pose),
                                        built_kit_product_type_pose_[part_type].end());

                        // increasing the counter here, because we only nee 'difference' number of parts to remove
                        i++;
                    }
                }
            }
        }else{
            // Enters here as the part type doesn't exist in the updated order
            // We have to remove all the parts with that part type
            std::cout << "[AriacSensorManager][WhatToRemove] : non existent parts type is : " << part_type << "\n";
            auto built_non_existent = built_kit_product_type_pose_[part_type];
            parts_to_remove_product_type_pose_[part_type] = built_non_existent;

            // Erase built_non_existent pose from the built map that will be thrown away.
            // Using the std::remove to find the part from the vector to remove it for built_map
            built_kit_product_type_pose_.erase(part_type);
            NumPartsToRemove += built_non_existent.size();
            // not found
        }
    }
}

// Second, Check for number of parts to be changed positions
void  AriacSensorManager::WhatToModify(){

    order_update_copy = order_update_product_type_pose_;

    // Looping through the maps
    for (auto const& built_item: built_kit_product_type_pose_){

        auto built_part_type = built_item.first;        //gear_part
        std::cout << "[AriacSensorManager][WhatToModify] : The part type is : " << built_part_type << "\n";

        for (auto built_pose : built_item.second){

            for (auto const& order_item: order_update_copy){

                auto order_part_type = order_item.first;

                for (auto update_pose : order_item.second){

                    if (built_pose == update_pose &&  built_part_type == order_part_type){

                        std::cout << "[AriacSensorManager][WhatToModify] : Poses match for same type, Lucky!! : " << "\n";
//                        std::cout << "[AriacSensorManager][WhatToModify] : order type!! : " << order_part_type << "\n";
//                        std::cout << "[AriacSensorManager][WhatToModify] : Built type!! : " << built_part_type << "\n";
//                        std::cout << "[AriacSensorManager][WhatToModify] : order pose!! : " << update_pose << "\n";
//                        std::cout << "[AriacSensorManager][WhatToModify] : Built pose!! : " << built_pose << "\n";

                        order_update_copy[order_part_type].erase(std::remove(order_update_copy[order_part_type].begin(),
                                order_update_copy[order_part_type].end(), update_pose),
                                        order_update_copy[order_part_type].end());

                        built_kit_product_type_pose_[built_part_type].erase(std::remove(built_kit_product_type_pose_[built_part_type].begin(),
                                built_kit_product_type_pose_[built_part_type].end(), built_pose),
                                        built_kit_product_type_pose_[built_part_type].end());

                    }else if(built_pose == update_pose &&  built_part_type != order_part_type){

                        std::cout << "[AriacSensorManager][WhatToModify] : Poses match with different type, More Work!! : " << "\n";
//                        std::cout << "[AriacSensorManager][WhatToModify] : order type!! : " << order_part_type << "\n";
//                        std::cout << "[AriacSensorManager][WhatToModify] : Built type!! : " << built_part_type << "\n";
//                        std::cout << "[AriacSensorManager][WhatToModify] : order pose!! : " << update_pose << "\n";
//                        std::cout << "[AriacSensorManager][WhatToModify] : Built pose!! : " << built_pose << "\n";

                        built_kit_product_type_pose_[built_part_type].erase(std::remove(built_kit_product_type_pose_[built_part_type].begin(),
                                built_kit_product_type_pose_[built_part_type].end(), built_pose),
                                        built_kit_product_type_pose_[built_part_type].end());
                        parts_to_remove_product_type_pose_[built_part_type].emplace_back(built_pose);
                        NumPartsToRemove += 1;

                    }
                }
            }
        }
    }

    for (auto const& built_item: built_kit_product_type_pose_) {
        auto built_part_type = built_item.first;
        NumPartsToModify = NumPartsToModify + built_item.second.size();
    }
}

void  AriacSensorManager::WhatToAdd(){

    std::cout << "[AriacSensorManager][WhatToAdd] : Inside the what to add : " << "\n";

    for(auto const& pair : order_update_copy){

        NumPartsToAdd += pair.second.size();
    }
}

void AriacSensorManager::removeParts(){

    int agv_id = 2;
    for(auto const& pair : parts_to_remove_product_type_pose_){
        std::cout << "[AriacSensorManager][removeParts] : part type to remove " << pair.first << std::endl;
        auto part_type = pair.first;

        for (auto part_pose : pair.second){
            std::cout << "[AriacSensorManager][removeParts] : part pose to remove " << part_pose << std::endl;

            geometry_msgs::Pose drop_pose = kitToWorld(part_pose, agv_id);

            arm2.SendRobotTo(arm2_check_qc_pose);
            ros::Duration(0.5).sleep();

            PickAndThrow(drop_pose, part_type, arm2);
            ros::Duration(0.5).sleep();

            // Erase part_pose that is thrown away. Using the std::remove to find the part from the vector
//             built_kit_product_type_pose_[part_type].erase(std::remove(built_kit_product_type_pose_[part_type].begin(),
//                    built_kit_product_type_pose_[part_type].end(), part_pose),
//                            built_kit_product_type_pose_[part_type].end());

        }
    }
}

void AriacSensorManager::modifyPose(){

    // we need to get the current pose and where to place
    // iterate through whatever is left on the Built_kit_map and search for that part_type in Order_update_map.
    // built_kit_map gives the current pose and order_update_kit gives the updated pose.

    int agv_id = 2;

    for (auto const& built_item: built_kit_product_type_pose_){

        auto built_part_type = built_item.first;
        std::cout << "[AriacSensorManager][modifyPose] : The part type is : " << built_part_type << "\n";

        for (auto built_pose : built_item.second) {

            auto order_vect = order_update_copy[built_part_type];

            auto update_pose = order_vect.back();
            order_vect.pop_back();

            // use pickandplace, here built_pose is current pose, update_pose is the updated pose where the
            // parts needs to be placed
            CorrectPose(built_pose, update_pose, built_part_type, agv_id);

            order_update_copy[built_part_type].erase(std::remove(order_update_copy[built_part_type].begin(),
                    order_update_copy[built_part_type].end(), update_pose),
                            order_update_copy[built_part_type].end());

            built_kit_product_type_pose_[built_part_type].erase(std::remove(built_kit_product_type_pose_[built_part_type].begin(),
                    built_kit_product_type_pose_[built_part_type].end(), built_pose),
                    built_kit_product_type_pose_[built_part_type].end());
        }
    }
}

void AriacSensorManager::addParts(){

    int agv_id = 2;

    std::cout << "[AriacSensorManager][AddParts] : Inside the AddParts : "  << "\n";

    for(auto const& pair : order_update_copy){

        auto part_type = pair.first;

        for(auto part_pose : pair.second){

            std::pair<std::string,geometry_msgs::Pose> type_pose_pair;
            type_pose_pair.first = part_type;
            type_pose_pair.second = part_pose;
            PickAndPlace(type_pose_pair, agv_id);

        }
    }
}

void AriacSensorManager::ReExecute(){

    int agv_id = 2;

    std::cout << "[AriacSensorManager][ReExecute] : Inside the ReExecute : "  << "\n";

    for(auto const& pair : order_update_product_type_pose_){

        auto part_type = pair.first;

        for(auto part_pose : pair.second){

            std::pair<std::string,geometry_msgs::Pose> type_pose_pair;
            type_pose_pair.first = part_type;
            type_pose_pair.second = part_pose;
            PickAndPlace(type_pose_pair, agv_id);

        }
    }
}

void AriacSensorManager::UpdateKit(){

    // First built a map for new updated orders --> may be we should do this in the order callback.

    // First, Check for number of Parts to be removed.
    WhatToRemove();

    for(auto const& pair : parts_to_remove_product_type_pose_){
        std::cout << "[AriacSensorManager][UpdateKit] : Parts to remove:  part type is " << pair.first << std::endl;
        auto parts = parts_to_remove_product_type_pose_[pair.first];

        for (auto pa : parts){
            std::cout << "[AriacSensorManager][UpdateKit] : Parts to remove: part pose is " << pa << std::endl;
        }
    }

    std::cout << "[AriacSensorManager][UpdateKit] : Total Number of Parts to remove " << NumPartsToRemove << std::endl;

    for(auto const& pair : built_kit_product_type_pose_){
        std::cout << "[AriacSensorManager][UpdateKit] : After removal part type in built_kit is " << pair.first << std::endl;
        auto parts = built_kit_product_type_pose_[pair.first];

        for (auto pa : parts){
            std::cout << "[AriacSensorManager][UpdateKit] : After removal part pose in built_kit is " << pa << std::endl;
        }
    }

    // Second, Check for number of parts to be changed positions
    WhatToModify();

    std::cout << "[AriacSensorManager][UpdateKit] : Total Number of Parts to Modify " << NumPartsToModify << std::endl;
    std::cout << "[AriacSensorManager][UpdateKit] : Total Number of Parts to remove after WhatToModify " << NumPartsToRemove << std::endl;

    for(auto const& pair : built_kit_product_type_pose_){
        std::cout << "[AriacSensorManager][UpdateKit] : After whatToModify part type in built_kit is " << pair.first << std::endl;
        auto parts = built_kit_product_type_pose_[pair.first];

        for (auto pa : parts){
            std::cout << "[AriacSensorManager][UpdateKit] : After whatToModify part pose in built_kit is " << pa << std::endl;
        }
    }

    // Third, Check for number of Parts to be Added.
    WhatToAdd();

    int TotalNumberOfChanges = NumPartsToRemove + NumPartsToModify + NumPartsToAdd;
    int threshold = 1;

    // Find the total number of changes to be done and compare with a threshold. Based on this comparison
    // decide whether to proceed with the changes/ build a kit

    // If number of changes is less than threshold:
    if (TotalNumberOfChanges < threshold){

        // Call the function to removePart()
        removeParts();

        // Call the function to modifyPosition()
        modifyPose();

        for(auto const& pair : order_update_copy){
            std::cout << "[AriacSensorManager][UpdateKit] : After whatToModify part type in order_update is " << pair.first << std::endl;
            auto parts = order_update_copy[pair.first];

            for (auto pa : parts){
                std::cout << "[AriacSensorManager][UpdateKit] : After whatToModify part type in order_update is " << pa << std::endl;
            }
        }

        // Call the function to addParts()
        addParts();
    }else{

        SubmitAGV(2);

        ros::Duration(15.0).sleep();

        ReExecute();

    }

    // Else if number of changes is greater that threshold:
    // Call the function to ExecuteOrder() for order_0_update_0
}

void AriacSensorManager::SubmitAGV(int num) {
    std::string s = std::to_string(num);
    ros::ServiceClient start_client =
            sensor_nh_.serviceClient<osrf_gear::AGVControl>("/ariac/agv"+s);
    if (!start_client.exists()) {
        ROS_INFO("Waiting for the client to be ready...");
        start_client.waitForExistence();
        ROS_INFO("Service started.");
    }

    osrf_gear::AGVControl srv;
    // srv.request.kit_type = "order_0_kit_0";
    start_client.call(srv);

    if (!srv.response.success) {
        ROS_ERROR_STREAM("Service failed!");
    } else
        ROS_INFO("Service succeeded.");
}

void AriacSensorManager::qc_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    if (image_msg->models.size() == 0){
        qc_2_redFlag = false;
    }
    else {
        ROS_WARN_STREAM_ONCE("[qc_2_callback] detected faulty part");
        qc_2_redFlag = true;
    }
}

void AriacSensorManager::qc_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    if (image_msg->models.size() == 0)
        qc_1_redFlag = false;
    else{
        ROS_WARN_STREAM("[qc_1_callback]: faulty part detected");
        qc_1_redFlag = true;
    }
}
