#include "AriacSensorManager.h"
using namespace std;

AriacSensorManager::AriacSensorManager() :
    task {},
    arm1 {"arm1"},
    arm2 {"arm2"}
{
    orders_sub = sensor_nh_.subscribe("/ariac/orders", 10, 
        & AriacSensorManager::order_callback, this);

    bb_1_sub = sensor_nh_.subscribe("/ariac/break_beam_1_change", 10,
        & AriacSensorManager::bb_1_callback, this);

    bb_2_sub = sensor_nh_.subscribe("/ariac/break_beam_2_change", 10,
        & AriacSensorManager::bb_2_callback, this);

    lc_bin_1_sub = sensor_nh_.subscribe("/ariac/lc_bin_1", 10, 
        & AriacSensorManager::lc_bin_1_callback, this);

    lc_agv_1_sub = sensor_nh_.subscribe("/ariac/lc_agv_1", 10, 
        & AriacSensorManager::lc_agv_1_callback, this);

    order_number = 0;

    qc_1_redFlag = false;
    qc_2_redFlag = false;
    order_receiving_flag = false;
    arm1_busy = false;

    arm2_check_qc_pose["linear_arm_actuator_joint"] = -1.1;
    arm2_check_qc_pose["shoulder_pan_joint"] = 4.6;
    arm2_check_qc_pose["elbow_joint"] = 0;
    arm2_check_qc_pose["shoulder_lift_joint"] = 0;
    arm2_check_qc_pose["wrist_1_joint"] = 3.14 * 3 / 2;
    arm2_check_qc_pose["wrist_2_joint"] = 3.14/2;

    go_transition_pose["linear_arm_actuator_joint"] = -0.56;
    go_transition_pose["shoulder_pan_joint"] = 3.14;
    go_transition_pose["shoulder_lift_joint"] = -3.14/2;
    go_transition_pose["elbow_joint"] = 3.14/2;
    go_transition_pose["wrist_1_joint"] = 3.14 * 3 / 2;
    go_transition_pose["wrist_2_joint"] = 3.14/2;

    back_transition_pose["linear_arm_actuator_joint"] = -0.56;
    back_transition_pose["shoulder_pan_joint"] = 3.14;
    back_transition_pose["shoulder_lift_joint"] = -3.14/2;
    back_transition_pose["elbow_joint"] = 3.14/2;
    back_transition_pose["wrist_1_joint"] = -1.5;
    back_transition_pose["wrist_2_joint"] = -3.14/2;

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
    arm1_check_qc_pose["wrist_2_joint"] = 1.57;
}

AriacSensorManager::~AriacSensorManager() {}

void AriacSensorManager::order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    ROS_INFO_STREAM("[ASM]:[order_callback]:Received order:\n" << *order_msg);
    received_orders_.push_back(*order_msg);
    setDesiredParts();
    order_receiving_flag = true;
    ROS_INFO("[ASM]:[order_callback]:Subscribing to LC gear Bin topic");
    lc_gear_sub = sensor_nh_.subscribe("/ariac/lc_gear", 10, 
        & AriacSensorManager::lc_gear_callback, this);
}

void AriacSensorManager::setDesiredParts(){
    ROS_INFO_STREAM("[ASM]:[setDesiredParts]:Setting desired parts");
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
           ++(task[product.type]); // Adding product name in task variable
        }
    }

    ROS_INFO_STREAM("[ASM]:[setDesiredParts]:The current desired_parts are:");
    for (const auto & part : desired_parts_info){
        std::cout << part.first << std::endl;
        std::cout << "Pose:\n";
        ROS_INFO_STREAM(part.second);
    }
    if (!order_id.empty())
        ++order_number;
}

void AriacSensorManager::lc_gear_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    if (image_msg->models.size() == 0){
        ROS_WARN_THROTTLE(5, "[ASM]:[lc_gear_callback]: lc_gear does not detect things");
        return;
    }

    ros::AsyncSpinner spinner(0);
    spinner.start();

    ROS_INFO_STREAM_THROTTLE(5, "[ASM]:[lc_gear_callback]: '" << image_msg->models.size() << "' gears.");
    if (order_receiving_flag){
        lc_gear_sub.shutdown();
        ROS_INFO_STREAM("[ASM]:[lc_gear_callback]: lc_gear_callback shutdown until next order");
        gear_check(image_msg);
    }
    // ros::spinOnce();
}

void AriacSensorManager::gear_check(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::Duration timeout(0.2);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    size_t desired_gear_num = task["gear_part"]; // count number of gears needed
    size_t gear_counter = image_msg->models.size(); // for labeling number of gear part
    for (auto & msg : image_msg->models){
        geometry_msgs::TransformStamped transformStamped;
        string camera_frame = "lc_gear_" + msg.type + "_" + to_string(gear_counter) + "_frame";
        ROS_INFO_STREAM("[ASM]:[gear_check]:The frame is named: " << camera_frame);
        try{
            transformStamped = tfBuffer.lookupTransform("world", camera_frame, ros::Time(0), timeout);  
            geometry_msgs::Pose part_pose; 
            part_pose.position.x = transformStamped.transform.translation.x;
            part_pose.position.y = transformStamped.transform.translation.y;
            part_pose.position.z = transformStamped.transform.translation.z;
            part_pose.orientation.x = transformStamped.transform.rotation.x;
            part_pose.orientation.y = transformStamped.transform.rotation.y;
            part_pose.orientation.z = transformStamped.transform.rotation.z;
            part_pose.orientation.w = transformStamped.transform.rotation.w;

            if (task["gear_part"] != 0){
                bool if_pick = arm2.PickPart(part_pose);
                if (if_pick) {
                    arm2.SendRobotTo(go_transition_pose);
                    arm2.SendRobotTo(arm2_check_qc_pose);
                    qc_2_sub = sensor_nh_.subscribe("/ariac/quality_control_sensor_2", 1, 
                        & AriacSensorManager::qc_2_callback, this);
                    if (qc_2_redFlag) { 
                        // if the one below the camera is a faulty one
                        ROS_INFO_STREAM("[ASM]:[gear_check]: QC 2 detected bad shit, ready to dispose....");
                        arm2.SendRobotTo("shoulder_pan_joint", 3.9);
                        arm2.GripperToggle(false);
                        arm2.SendRobotTo(back_transition_pose);
                        arm2.RobotGoHome();
                    }
                    else { 
                        // if the one below the camera is a good one
                        arm2.SendRobotTo(back_transition_pose); 
                        arm2.RobotGoHome();
                        bool attach = arm2.DropPart(part_pose);
                        // If part is dropped now, push it in gear_bin_vector
                        if (!attach)
                            gear_bin_vector.push_back(part_pose); // Storing the pose of the good gears inside gear_bin_vector
                        else
                            ROS_INFO_STREAM("[ASM][gear_check]: DropPart() states it was unsuccessful in dropping");
                    }
                    qc_2_sub.shutdown(); // unsubscirbe to qc_2
                }
                --(task["gear_part"]);
            }
            --gear_counter;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s\n",ex.what());
        }
    }

    arm2.SendRobotTo(back_transition_pose);
    ros::Duration(1).sleep();
    ROS_INFO("[ASM]:[gear_check]: Called grab_gear()");
    grab_gear();
}

void AriacSensorManager::qc_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    if (image_msg->models.size() == 0){
        // ROS_WARN_STREAM("QC 2 doesn't see shit");
        qc_2_redFlag = false;
    }
    else{
        ROS_WARN_STREAM("[ASM]:[qc_2_callback]: Bad thing detected");
        qc_2_redFlag = true;
    }
}

void AriacSensorManager::bb_1_callback(const osrf_gear::Proximity::ConstPtr & msg) {
    ros::AsyncSpinner spinner(0);
    spinner.start();
    if (msg->object_detected){
        lc_belt_sub = sensor_nh_.subscribe("/ariac/lc_belt", 10, 
            & AriacSensorManager::lc_belt_callback, this);
    }
}

void AriacSensorManager::lc_belt_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    if (image_msg->models.size() == 0) return;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::Duration timeout(0.2);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    for (auto & msg : image_msg->models){
        geometry_msgs::TransformStamped transformStamped;

        // find if this type of part has passed by before
        if (belt_part_counter.find(msg.type) == belt_part_counter.end())
            belt_part_counter.insert(make_pair(msg.type, 0));
        else
            belt_part_counter[msg.type]++;
        
        string camera_frame = "lc_belt_" + msg.type + "_" + 
            to_string(belt_part_counter[msg.type]) + "_frame";
        try{
            transformStamped = tfBuffer.lookupTransform("world", camera_frame, ros::Time(0), timeout);
            std::pair<std::string, std::string> part_pair {msg.type, camera_frame};
            incoming_partQ.push(part_pair);
            geometry_msgs::Pose part_pose; 
            part_pose.position.x = transformStamped.transform.translation.x;
            // part_pose.position.y = transformStamped.transform.translation.y;
            part_pose.position.y = 1.65;
            part_pose.position.z = transformStamped.transform.translation.z;
            part_pose.orientation.x = transformStamped.transform.rotation.x;
            part_pose.orientation.y = transformStamped.transform.rotation.y;
            part_pose.orientation.z = transformStamped.transform.rotation.z;
            part_pose.orientation.w = transformStamped.transform.rotation.w;
            belt_part_map.insert({camera_frame, part_pose});
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s\n",ex.what());
            belt_part_counter[msg.type]--;
        }
        break;
    }
    lc_belt_sub.shutdown();
}

void AriacSensorManager::bb_2_callback(const osrf_gear::Proximity::ConstPtr & msg) {
    ros::AsyncSpinner spinner(0);
    spinner.start();
    // ROS_INFO_STREAM(">>> Inside bb_2_callback");
    if (msg->object_detected) {
        auto incoming_part = incoming_partQ.front(); incoming_partQ.pop();
        ROS_INFO_STREAM("[ASM]:[bb_2_callback]: Break beam triggered. The part is '" << incoming_part.first << "'");
        if (!arm1_busy) {
            // if (desired_parts.find(incoming_part.first) != desired_parts.end() 
            //     && incoming_part.first == "piston_rod_part")
            if ((task[incoming_part.first] != 0) && 
                (incoming_part.first == "piston_rod_part"))
            {
                ROS_INFO_STREAM("[ASM]:[bb_2_callback]: Pick this part!");
                arm1_busy = true;
                pick_part_from_belt(incoming_part);
            } 
            else {
                ROS_INFO_STREAM("[ASM]:[bb_2_callback]: Doesn't need this part ...");
            }
        }
        else {
            ROS_INFO_STREAM("[ASM]:[bb_2_callback]: Arm1 is busy...");
        }
    }
}

void AriacSensorManager::pick_part_from_belt(pair<string, string> incoming_part){
//    ros::AsyncSpinner spinner(0);
//    spinner.start();
    ROS_INFO_STREAM("[ASM]:[pick_part_from_belt]: Inside pick_part_from_belt");
    auto part_frame = belt_part_map[incoming_part.second];

    bool if_pick = arm1.PickPart(part_frame);
    if (if_pick){
        qc_1_sub = sensor_nh_.subscribe("/ariac/quality_control_sensor_1", 1, 
            & AriacSensorManager::qc_1_callback, this);
        arm1.SendRobotTo(arm1_check_qc_pose);
        qc_1_sub.shutdown();
        if (qc_1_redFlag) { // if the one below the camera is a faulty one
            ROS_INFO_STREAM("[ASM]:[pick_part_from_belt]: QC 1 detected bad shit, ready to dispose....");
            arm1_check_qc_pose["shoulder_pan_joint"] = 2.32;
            arm1.SendRobotTo(arm1_check_qc_pose);
            arm1.GripperToggle(false);
            // lc_bin_1_sub = sensor_nh_.subscribe("/ariac/lc_bin_1", 10, 
            //     & AriacSensorManager::lc_bin_1_callback, this);
        }
        else { // if the one below the camera is a good one
            arm1.SendRobotTo(arm1_bin_pose);
            arm1.GripperToggle(false);
            --(task[incoming_part.first]);
        }
        arm1.RobotGoHome();
    }
    arm1_busy = false;
}

void AriacSensorManager::qc_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    if (image_msg->models.size() == 0)
        qc_1_redFlag = false;
    else{
        ROS_WARN_STREAM("[ASM]:[qc_1_callback]: Bad thing detected");
        qc_1_redFlag = true;
    }
}

void AriacSensorManager::lc_bin_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    if (image_msg->models.size() == 0){
        ROS_WARN_THROTTLE(5, "[ASM]:[lc_bin_1_callback]: Bin 1 sees no part");
        return;
    }

    ros::AsyncSpinner spinner(0);
    spinner.start();
    ROS_INFO_STREAM_THROTTLE(5, "[ASM]:[lc_bin_1_callback]: lc_bin captures '" << image_msg->models.size() << "' item(s).");
    if (!arm1_busy){
        ROS_INFO_STREAM("[ASM]:[lc_bin_1_callback]: Calling function grab_bin1");
        grab_bin1(image_msg);
        // lc_bin_1_sub.shutdown();
    }
}

void AriacSensorManager::grab_bin1(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    ROS_INFO_STREAM("[ASM]:[grab_bin1]: Inside function grab_bin1");

    ros::AsyncSpinner spinner(0);
    spinner.start();
    
    ros::Duration timeout(0.2);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    size_t part_counter = 0; // for labeling number of gear part // --- Issue

    for (auto & msg : image_msg->models){
        geometry_msgs::TransformStamped transformStamped;
        string bin_part_camera_frame = "lc_bin_1_" + msg.type + "_" + to_string(part_counter) + "_frame";
        ROS_INFO_STREAM("[ASM]:[grab_bin1]: The frame is named: " << bin_part_camera_frame);
        try{
            transformStamped = tfBuffer.lookupTransform("world", bin_part_camera_frame, ros::Time(0), timeout);
            geometry_msgs::Pose part_pose; 
            part_pose.position.x = transformStamped.transform.translation.x;
            part_pose.position.y = transformStamped.transform.translation.y;
            part_pose.position.z = transformStamped.transform.translation.z;
            part_pose.orientation.x = transformStamped.transform.rotation.x;
            part_pose.orientation.y = transformStamped.transform.rotation.y;
            part_pose.orientation.z = transformStamped.transform.rotation.z;
            part_pose.orientation.w = transformStamped.transform.rotation.w;

            if (!arm1_busy && (desired_parts_info.find(msg.type) != desired_parts_info.end())) {
                arm1_busy = true;
                arm1.SendRobotTo(arm1_bin_pose);
                bool if_pick = arm1.PickPart(part_pose);
                if (if_pick) {
                    ROS_INFO_STREAM("[ASM]:[grab_bin1]: Try FK");
                    arm1.SendRobotTo(arm1_bin_pose);
                    try {
                        auto itr = desired_parts_info.find(msg.type);
                        auto drop_pose_ = itr->second;
                        geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
                        StampedPose_in.header.frame_id = "kit_tray_1";
                        StampedPose_in.pose = drop_pose_;
                        StampedPose_out = tfBuffer.transform(StampedPose_in, "world");
                        auto drop_pose = StampedPose_out.pose;
                        drop_pose.position.z += 0.05;
                        ROS_INFO_STREAM("[ASM]:[grab_bin1]: Drop pose from tf2 transform:\n" << drop_pose);

                        bool attach = arm1.DropPart(drop_pose);
                        arm1.RobotGoHome();
                        desired_parts_info.erase(itr);
                    }
                    catch (tf2::TransformException &ex) {
                        ROS_INFO_STREAM("[ASM]:[grab_bin1]: grab_bin1 error");
                        ROS_WARN("%s\n", ex.what());
                    }
                }
                arm1_busy = false;
            ++part_counter;
            }
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s\n",ex.what());
            ROS_INFO_STREAM("[ASM]:[grab_bin_1]: Frame Name Lookup Error, Frame name received:" << bin_part_camera_frame);
        }
    }
}

void AriacSensorManager::lc_agv_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
    ros::AsyncSpinner spinner(0);
    spinner.start();

    if (image_msg->models.size() == 0){
        ROS_INFO_STREAM_THROTTLE(5, "[ASM]:[lc_agv_1_callback]: lc_agv_1 does not detect things");
        return;
    }

    ROS_INFO_STREAM_THROTTLE(5, "[ASM]:[lc_agv_1_callback]: Inside lc_agv_1_callback");

    ROS_INFO_STREAM_THROTTLE(5, "[ASM]:[lc_agv_1_callback]: lc_agv_1 captures '" << image_msg->models.size() << "' item(s).");
    
    ros::Duration timeout(0.2);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    size_t part_counter = 0;
    for (auto & msg : image_msg->models){
        geometry_msgs::TransformStamped transformStamped;
        string camera_frame = "lc_agv_1_" + msg.type + "_" + to_string(part_counter) + "_frame";
        ROS_INFO_STREAM("[ASM]:[lc_agv_1_callback]: The frame is named: " << camera_frame);
        try{
            transformStamped = tfBuffer.lookupTransform("world", camera_frame, ros::Time(0), timeout);
            geometry_msgs::Pose part_pose; 
            part_pose.position.x = transformStamped.transform.translation.x;
            part_pose.position.y = transformStamped.transform.translation.y;
            part_pose.position.z = transformStamped.transform.translation.z;
            part_pose.orientation.x = transformStamped.transform.rotation.x;
            part_pose.orientation.y = transformStamped.transform.rotation.y;
            part_pose.orientation.z = transformStamped.transform.rotation.z;
            part_pose.orientation.w = transformStamped.transform.rotation.w;  

            geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
            StampedPose_in.header.frame_id = "world";
            StampedPose_in.pose = part_pose;
            StampedPose_out = tfBuffer.transform(StampedPose_in, "kit_tray_1");
            auto agv_pose = StampedPose_out.pose;

            ROS_INFO_STREAM("[ASM]:[lc_agv_1_callback]: Part pose on the tray in kit_tray_1 frame is:\n" << part_pose);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s\n",ex.what());
        }
    }
}

void AriacSensorManager::grab_gear(){
    ROS_INFO_STREAM("[ASM]:[grab_gear]: Inside function grab_gear");

    ros::AsyncSpinner spinner(0);
    spinner.start();
    
    ros::Duration timeout(0.2);
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    size_t part_counter = 0; // for labeling number of gear part
    ROS_INFO_STREAM("[ASM]:[grab_gear]: gear_bin_vector contains: " << gear_bin_vector.size() << " Gears");
    for (auto & msg : gear_bin_vector){
//        ROS_INFO_STREAM("[ASM]:[grab_gear]: inside the for loop");
        ROS_INFO_STREAM("[ASM]:[grab_gear]: Pose to pick part from:" << msg);

        try{
            geometry_msgs::Pose part_pose = msg;
            if (!arm1_busy && (desired_parts_info.find("gear_part") != desired_parts_info.end())){
                arm1_busy = true;
                arm1.SendRobotTo(arm1_bin_pose);
                bool if_pick = arm1.PickPart(part_pose);
                if (if_pick) {
                    ROS_INFO_STREAM("[ASM]:[grab_gear]: Try FK");
                    arm1.SendRobotTo(arm1_bin_pose);
                    try{
                        auto itr = desired_parts_info.find("gear_part");
                        auto drop_pose_ = itr->second;
                        geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
                        StampedPose_in.header.frame_id = "kit_tray_1";
                        StampedPose_in.pose = drop_pose_;
                        StampedPose_out = tfBuffer.transform(StampedPose_in, "world");
                        auto drop_pose = StampedPose_out.pose;
                        drop_pose.position.z += 0.05;
                        ROS_INFO_STREAM("[ASM]:[grab_gear]: Drop pose from tf2 transform:\n" << drop_pose);

                        bool attach = arm1.DropPart(drop_pose);
                        arm1.RobotGoHome();
                        desired_parts_info.erase(itr);
                    }
                    catch (tf2::TransformException &ex) {
                        ROS_INFO_STREAM("[ASM]:[grab_gear]: grab_bin1 error");
                        ROS_WARN("%s\n",ex.what());
                    }
                }
                arm1_busy = false;
            }
            ++part_counter;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s\n",ex.what());
        }
    }
}