#include "AriacSensorManager.h"
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

    init_ = false;
    cam_1_ = false;
    cam_2_ = false;
    cam_3_ = false;
    cam_4_ = false;
    cam_5_ = false;
    cam_6_ = false;

    camera1_frame_counter_ = 1;
    camera2_frame_counter_ = 1;
    camera3_frame_counter_ = 1;
    camera4_frame_counter_ = 1;
    camera5_frame_counter_ = 1;
    camera6_frame_counter_ = 1;


    order_number = 0;
    int counter = 0;

    qc_1_redFlag = false;
    qc_2_redFlag = false;
    order_receiving_flag = false;
    arm1_busy = false;
    everything_ready = false;

    arm2_check_qc_pose["linear_arm_actuator_joint"] = -1.1;
    arm2_check_qc_pose["shoulder_pan_joint"] = 4.6;
    arm2_check_qc_pose["elbow_joint"] = 0;
    arm2_check_qc_pose["shoulder_lift_joint"] = 0;
    arm2_check_qc_pose["wrist_1_joint"] = -1.57;
    arm2_check_qc_pose["wrist_2_joint"] = -3.14/2;

    arm2_transition_pose["linear_arm_actuator_joint"] = -0.56;
    arm2_transition_pose["shoulder_pan_joint"] = 3.14;
    arm2_transition_pose["shoulder_lift_joint"] = -3.14/2;
    arm2_transition_pose["elbow_joint"] = 3.14/2;
    arm2_transition_pose["wrist_1_joint"] = -1.5;
    arm2_transition_pose["wrist_2_joint"] = -3.14/2;

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
    ROS_INFO_STREAM("[order_callback]:Received order:\n" << *order_msg);
    received_orders_.push_back(*order_msg);
    setDesiredParts();
    order_receiving_flag = true;

//    lc_bin_1_sub = sensor_nh_.subscribe("/ariac/lc_bin_1", 10,
//                                        & AriacSensorManager::lc_bin_1_callback, this);
//
//    lc_bin_2_sub = sensor_nh_.subscribe("/ariac/lc_bin_2", 10,
//                                        &AriacSensorManager::lc_bin_2_callback, this);
//    lc_bin_3_sub = sensor_nh_.subscribe("/ariac/lc_bin_3", 10,
//                                        &AriacSensorManager::lc_bin_3_callback, this);
//
//    lc_bin_4_sub = sensor_nh_.subscribe("/ariac/lc_bin_4", 10,
//                                        &AriacSensorManager::lc_bin_4_callback, this);
//
//    lc_bin_5_sub = sensor_nh_.subscribe("/ariac/lc_bin_5", 10,
//                                        &AriacSensorManager::lc_bin_5_callback, this);
//
//    lc_bin_6_sub = sensor_nh_.subscribe("/ariac/lc_bin_6", 10,
//                                        &AriacSensorManager::lc_bin_6_callback, this);

//    lc_gear_sub = sensor_nh_.subscribe("/ariac/lc_gear", 10,
//                                       & AriacSensorManager::lc_gear_callback, this);


}

//void AriacSensorManager::lc_bin_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
//    if (image_msg->models.size() == 0)
//        return;
//
//    ros::AsyncSpinner spinner(4);
//    spinner.start();
//    ROS_INFO_STREAM_THROTTLE(5, "[lc_bin_1_callback]: lc_bin captures '" << image_msg->models.size() << "' item(s).");
//    if (!arm1_busy)
//        grab_bin1(image_msg);
//}

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
//    PickAndPlace();


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



bool AriacSensorManager::PickAndPlace(const std::pair<std::string,geometry_msgs::Pose> product_type_pose, int agv_id){

    ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlace]:Inside PickAndPlace ...");

    if(init_){

//        std::string product_type = "pulley_part";
//        auto vect = product_frame_list_[product_type];
//        std::string product_frame = vect.back();

        std::string product_type = product_type_pose.first;

        ROS_WARN_STREAM("[AriacSensorManager]:[PickAndPlace]:Product type >>>> " << product_type);
        std::string product_frame = this->GetProductFrame(product_type);

        ROS_WARN_STREAM("Product frame >>>> " << product_frame);
        auto part_pose = GetPartPose("/world", product_frame);

        if (product_type == "pulley_part")
            part_pose.position.z += 0.08;
        //--task the robot to pick up this part
        bool failed_pick = arm2.PickPart(part_pose);
        ROS_WARN_STREAM("Picking up state " << failed_pick);
        ros::Duration(0.5).sleep();

        while (!failed_pick) {
            auto part_pose = GetPartPose("/world", product_frame);
            failed_pick = arm2.PickPart(part_pose);
        }

        // Here we will do Quality Check

//        arm2.RobotGoHome();

        ros::Duration(0.5).sleep();

        arm2_transition_pose["linear_arm_actuator_joint"] = -0.21;
        arm2_transition_pose["shoulder_pan_joint"] = 3.14;
        arm2_transition_pose["shoulder_lift_joint"] = -1;
        arm2_transition_pose["elbow_joint"] = 2.01;
        arm2_transition_pose["wrist_1_joint"] = -2.51;
        arm2_transition_pose["wrist_2_joint"] = -1.51;
        arm2_transition_pose["wrist_3_joint"] = 0;


        arm2.SendRobotTo(arm2_transition_pose);

        ros::Duration(1).sleep();

        agv_id = 2;

        // Function to get the drop pose in world coordinates
        geometry_msgs::Pose drop_pose = kitToWorld(product_type_pose.second, agv_id);


        ROS_INFO_STREAM("[AriacSensorManager]:[PickAndPlace]: Drop Pose : " << drop_pose);



        drop_pose.position.z += 0.05;
//        geometry_msgs::Pose drop_pose = product_type_pose.second;
//        drop_pose.position.x = -0.230428;
//        drop_pose.position.y = 0.375809;
//        drop_pose.position.z = 1.024050;
//
//        drop_pose.orientation.x = 0;
//        drop_pose.orientation.y = 0;
//        drop_pose.orientation.z = 0;
//        drop_pose.orientation.w = 0;

        auto result = arm2.DropPart(drop_pose);

        ros::Duration(1.5).sleep();

        arm2.SendRobotTo(arm2_transition_pose);

    }

}

//
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
                //--todo: What do we do if pick and place fails?
            }
//            SubmitAGV(2);
            ROS_INFO_STREAM("Submitting AGV 1");
            int finish=1;
        }

    }
}
//


//void AriacSensorManager::lc_gear_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
//    if (image_msg->models.size() == 0){
//        ROS_WARN_THROTTLE(5, "[lc_gear_callback]: lc_gear does not detect things");
//        return;
//    }
//    else{
//        parts_to_pickup_belt.erase("gear_part");
//    }
//
//
//    ros::AsyncSpinner spinner(4);
//    spinner.start();
//
//    ROS_INFO_STREAM_THROTTLE(5, "[lc_gear_callback]: '" << image_msg->models.size() << "' gears.");
//    if (order_receiving_flag){
//        lc_gear_sub.shutdown();
//        gear_check(image_msg);
//    }
//}


void AriacSensorManager::qc_2_callback(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
//    ros::AsyncSpinner spinner(4);
//    spinner.start();

    if (image_msg->models.size() == 0){
        qc_2_redFlag = false;
    }
    else {
        ROS_WARN_STREAM_ONCE("[qc_2_callback] detecte faulty part");
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

void AriacSensorManager::gear_check(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
//    ros::AsyncSpinner spinner(4);
//    spinner.start();

    ros::Duration timeout(0.2);

    size_t desired_gear_num = task["gear_part"]; // count number of gears needed
    size_t gear_counter = image_msg->models.size(); // for labeling number of gear part
    for (auto & msg : image_msg->models){
        geometry_msgs::TransformStamped transformStamped;
        string camera_frame = "lc_gear_" + msg.type + "_" + to_string(gear_counter) + "_frame";
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
                    /// ---------checking mechanism --------------------
                    qc_2_sub = sensor_nh_.subscribe("/ariac/quality_control_sensor_2", 10,
                                                    & AriacSensorManager::qc_2_callback, this);
                    arm2.SendRobotTo(arm2_transition_pose);
                    arm2.SendRobotTo(arm2_check_qc_pose);
                    arm2.GripperToggle(false); // release gripper

                    transformStamped = tfBuffer.lookupTransform("world", "arm2_ee_link", ros::Time(0), timeout);
                    geometry_msgs::Pose ee2_pose;
                    ee2_pose.position.x = transformStamped.transform.translation.x;
                    ee2_pose.position.y = transformStamped.transform.translation.y;
                    ee2_pose.position.z = transformStamped.transform.translation.z;
                    ee2_pose.position.z -= 0.275;

                    arm2.SendRobotTo("shoulder_pan_joint", 4.3);
                    ros::Duration(0.5).sleep();
                    qc_2_sub.shutdown();
                    arm2.SendRobotTo("shoulder_pan_joint", 4.6);

                    bool if_pick_tray = arm2.PickPart(ee2_pose);
                    ROS_INFO_STREAM("qc_2_redFlag = " << qc_2_redFlag);
                    while (!if_pick_tray) {
                        ee2_pose.position.z -= 0.05;
                        if_pick_tray = arm2.PickPart(ee2_pose);
                    }
                    /// ---------checking mechanism --------------------

                    if (qc_2_redFlag) {
                        // if the one below the camera is a faulty one
                        ROS_WARN_STREAM("[gear_check]: QC 2 detected faulty part, ready to dispose....");
                        arm2.SendRobotTo("shoulder_pan_joint", 3.9);
                        arm2.GripperToggle(false);
                        arm2.SendRobotTo(arm2_transition_pose);
                        arm2.RobotGoHome();
                    }
                    else {
                        // if the one below the camera is a good one
                        arm2.SendRobotTo(arm2_transition_pose);
                        arm2.RobotGoHome();
                        bool attach = arm2.DropPart(part_pose);
                        // If part is dropped now, push it in gear_bin_vector
                        if (!attach){
                            gear_bin_vector.push_back({camera_frame.substr(8), part_pose}); // Storing the pose of the good gears inside gear_bin_vector
                            --(task["gear_part"]);
                            ROS_INFO_STREAM("Need to grab '" << task["gear_part"] << "' gear");
                        }
                    }
                    qc_2_redFlag = false;
                }
            }
            --gear_counter;
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s\n",ex.what());
        }
    }
    arm2.RobotGoHome();
    ros::Duration(0.2).sleep();
    arm2.SendRobotTo(arm2_transition_pose);
    // arm2.SendRobotTo("linear_arm_actuator_joint", -1);
    ros::Duration(1).sleep();
    // ROS_INFO("[gear_check]: Called grab_gear()");
    grab_gear();
}

void AriacSensorManager::grab_bin1(const osrf_gear::LogicalCameraImage::ConstPtr& image_msg){
    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Duration timeout(0.2);

    size_t part_counter = 0; // for labeling number of gear part // --- Issue

    for (auto & msg : image_msg->models){
        geometry_msgs::TransformStamped transformStamped;
        string frame_name = popped_incoming_part.second;
        string bin_part_camera_frame = "lc_bin_1_" + frame_name;
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
                    ///------------- Get orientation of the bin in kit_tray frame -----------
                    geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
                    StampedPose_in.header.frame_id = "world";
                    StampedPose_in.pose = part_pose;
                    StampedPose_out = tfBuffer.transform(StampedPose_in, "kit_tray_1");
                    geometry_msgs::Pose part_pose_kit = StampedPose_out.pose;

                    tf2::Quaternion quat_tf;
                    tf2::fromMsg(part_pose_kit.orientation, quat_tf);
                    double part_R, part_P, part_Y;
                    tf2::Matrix3x3(quat_tf).getRPY(part_R, part_P, part_Y);

                    // ROS_INFO_STREAM("Part roll in kit_tray_frame = " << part_R);
                    // ROS_INFO_STREAM("Part pitch in kit_tray_frame = " << part_P);
                    // ROS_INFO_STREAM("Part yall in kit_tray_frame= " << part_Y);

                    transformStamped = tfBuffer.lookupTransform("kit_tray_1", "arm1_ee_link", ros::Time(0),
                                                               timeout);
                    geometry_msgs::Pose ee_pose;
                    ee_pose.orientation.x = transformStamped.transform.rotation.x;
                    ee_pose.orientation.y = transformStamped.transform.rotation.y;
                    ee_pose.orientation.z = transformStamped.transform.rotation.z;
                    ee_pose.orientation.w = transformStamped.transform.rotation.w;

                    double ee_R, ee_P, ee_Y;
                    tf2::Quaternion quat_tf_ee;
                    tf2::fromMsg(ee_pose.orientation, quat_tf_ee);
                    tf2::Matrix3x3(quat_tf_ee).getRPY(ee_R, ee_P, ee_Y);

                    // ROS_INFO_STREAM("EE roll in kit_tray_frame = " << ee_R);
                    // ROS_INFO_STREAM("EE pitch in kit_tray_frame = " << ee_P);
                    // ROS_INFO_STREAM("EE yall in kit_tray_frame= " << ee_Y);

                    double part_ee_ang_diff = part_Y - ee_Y;
                    ///----------modification end ----------------------
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

                        ///--------Get Deisred RPY from kit_tray
                        tf2::Quaternion quat_D;
                        tf2::fromMsg(drop_pose_.orientation, quat_D);
                        double d_R, d_P, d_Y;
                        tf2::Matrix3x3(quat_D).getRPY(d_R, d_P, d_Y);

                        arm1.GoToTarget1(drop_pose);
                        ros::Duration(0.5).sleep();
                        double comp = d_Y -  part_ee_ang_diff;
                        arm1.SendRobotTo("wrist_3_joint", comp);
                        ros::Duration(1).sleep();
                        arm1.GripperToggle(false);
                        ///--------------------------------------------------------
                        // bool attach = arm1.DropPart(drop_pose);
                        arm1.RobotGoHome();
                        desired_parts_info.erase(itr);
                    }
                    catch (tf2::TransformException &ex) {
                        ROS_WARN("%s\n", ex.what());
                    }
                }
                arm1_busy = false;
                ++part_counter;
            }
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s\n",ex.what());
        }
    }
}

// void AriacSensorManager::lc_agv_1_callback(const osrf_gear::LogicalCameraImage::ConstPtr & image_msg){
//     ros::AsyncSpinner spinner(4);
//     spinner.start();

//     if (image_msg->models.size() == 0)
//         return;

//     ROS_INFO_STREAM_THROTTLE(5, "[lc_agv_1_callback]: lc_agv_1 captures '" << image_msg->models.size() << "' item(s).");

//     ros::Duration timeout(0.2);

//     size_t part_counter = 0;
//     for (auto & msg : image_msg->models){
//         geometry_msgs::TransformStamped transformStamped;
//         string camera_frame = "lc_agv_1_" + msg.type + "_" + to_string(part_counter) + "_frame";
//         try{
//             transformStamped = tfBuffer.lookupTransform("world", camera_frame, ros::Time(0), timeout);
//             geometry_msgs::Pose part_pose;
//             part_pose.position.x = transformStamped.transform.translation.x;
//             part_pose.position.y = transformStamped.transform.translation.y;
//             part_pose.position.z = transformStamped.transform.translation.z;
//             part_pose.orientation.x = transformStamped.transform.rotation.x;
//             part_pose.orientation.y = transformStamped.transform.rotation.y;
//             part_pose.orientation.z = transformStamped.transform.rotation.z;
//             part_pose.orientation.w = transformStamped.transform.rotation.w;

//             geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
//             StampedPose_in.header.frame_id = "world";
//             StampedPose_in.pose = part_pose;
//             StampedPose_out = tfBuffer.transform(StampedPose_in, "kit_tray_1");
//             auto agv_pose = StampedPose_out.pose;

//             ROS_INFO_STREAM("[lc_agv_1_callback]: Part pose on the tray in kit_tray_1 frame is:\n" << part_pose);
//         }
//         catch (tf2::TransformException &ex) {
//             ROS_WARN("%s\n",ex.what());
//         }
//     // }
// }

//
//void AriacSensorManager::pick_part_from_belt(pair<string, string> incoming_part){
//    // ros::AsyncSpinner spinner(4);
//    // spinner.start();
//    popped_incoming_part = incoming_part;
//    auto part_pose = belt_part_map[incoming_part.second];
//
//    bool if_pick = arm1.PickPart(part_pose);
//    if (if_pick){
//
//        /// ---------checking mechanism --------------------
//        qc_1_sub = sensor_nh_.subscribe("/ariac/quality_control_sensor_1", 10,
//                                        & AriacSensorManager::qc_1_callback, this);
//        arm1.SendRobotTo(arm1_check_qc_pose);
//        arm1.GripperToggle(false);
//        ros::Duration timeout(0.2);
//
//        geometry_msgs::TransformStamped transformStamped;
//        transformStamped = tfBuffer.lookupTransform("world", "arm1_ee_link", ros::Time(0), timeout);
//        geometry_msgs::Pose ee1_pose;
//        ee1_pose.position.x = transformStamped.transform.translation.x;
//        ee1_pose.position.y = transformStamped.transform.translation.y;
//        ee1_pose.position.z = transformStamped.transform.translation.z;
//        ee1_pose.position.z -= 0.15;
//
//        arm1.SendRobotTo("shoulder_pan_joint", 1.6);
//        ros::Duration(0.5).sleep();
//        qc_1_sub.shutdown();
//        arm1.SendRobotTo("shoulder_pan_joint", 1.44);
//
//        bool if_pick_tray = arm1.PickPart(ee1_pose);
//        ROS_INFO_STREAM("qc_1_redFlag = " << qc_1_redFlag);
//        while (!if_pick_tray) {
//            ee1_pose.position.z -= 0.015;
//            if_pick_tray = arm1.PickPart(ee1_pose);
//        }
//        /// ---------checking mechanism --------------------
//
//
//        if (qc_1_redFlag) { // if the one below the camera is a faulty one
//            ROS_INFO_STREAM("QC 1 detected a faulty part, ready to dispose....");
//            arm1.SendRobotTo("shoulder_pan_joint", 2.32);
//            arm1.GripperToggle(false);
//        }
//        else { // if the one below the camera is a good one
//            arm1.SendRobotTo(arm1_bin_pose);
//            arm1.GripperToggle(false);
//            --(task[incoming_part.first]);
//            parts_to_pickup_belt.erase(incoming_part.first);
//
//        }
//        qc_1_redFlag = false;
//        arm1.RobotGoHome();
//    }
//    arm1_busy = false;
//}


void AriacSensorManager::grab_gear(){
    ros::AsyncSpinner spinner(4);
    spinner.start();

    geometry_msgs::TransformStamped transformStamped;
    ros::Duration timeout(0.2);

    size_t part_counter = 0;
    for (auto & msg : gear_bin_vector){
        try{
            geometry_msgs::Pose part_pose = msg.second;
            if (!arm1_busy && (desired_parts_info.find("gear_part") != desired_parts_info.end())){
                arm1_busy = true;
                arm1.SendRobotTo(arm1_bin_pose);
                arm1.SendRobotTo("linear_arm_actuator_joint",-1.2);

                bool if_pick = arm1.PickPart(part_pose);
                if (if_pick) {
                    // get part pose in kit_tray_frame
                    geometry_msgs::PoseStamped StampedPose_in, StampedPose_out;
                    StampedPose_in.header.frame_id = "world";
                    StampedPose_in.pose = part_pose;
                    StampedPose_out = tfBuffer.transform(StampedPose_in, "kit_tray_1");
                    geometry_msgs::Pose part_pose_kit = StampedPose_out.pose;

                    // get part RPY in kit_tray_frame
                    tf2::Quaternion quat_tf;
                    tf2::fromMsg(part_pose_kit.orientation, quat_tf);
                    double part_R, part_P, part_Y;
                    tf2::Matrix3x3(quat_tf).getRPY(part_R, part_P, part_Y);

                    // get ee pose in kit_tray_frame
                    transformStamped = tfBuffer.lookupTransform("kit_tray_1", "arm1_ee_link", ros::Time(0),
                                                               timeout);
                    geometry_msgs::Pose ee_pose;
                    ee_pose.orientation.x = transformStamped.transform.rotation.x;
                    ee_pose.orientation.y = transformStamped.transform.rotation.y;
                    ee_pose.orientation.z = transformStamped.transform.rotation.z;
                    ee_pose.orientation.w = transformStamped.transform.rotation.w;

                    // get ee RPY in kit_tray_frame
                    double ee_R, ee_P, ee_Y;
                    tf2::Quaternion quat_tf_ee;
                    tf2::fromMsg(ee_pose.orientation, quat_tf_ee);
                    tf2::Matrix3x3(quat_tf_ee).getRPY(ee_R, ee_P, ee_Y);

                    double part_ee_ang_diff = part_Y - ee_Y;
                    ///----------modification end ----------------------

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


                        ///--------Get Deisred RPY from kit_tray
                        tf2::Quaternion quat_D;
                        tf2::fromMsg(drop_pose_.orientation, quat_D);
                        double d_R, d_P, d_Y;
                        tf2::Matrix3x3(quat_D).getRPY(d_R, d_P, d_Y);   

                        arm1.GoToTarget1(drop_pose);
                        ros::Duration(0.5).sleep();
                        double comp = -(d_Y -  part_ee_ang_diff);
                        arm1.SendRobotTo("wrist_3_joint", comp);
                        ros::Duration(1).sleep();
                        arm1.GripperToggle(false);
                        ///--------------------------------------------------------


                        // bool attach = arm1.DropPart(drop_pose);
                        desired_parts_info.erase(itr);
                    }
                    catch (tf2::TransformException &ex) {
                        // ROS_INFO_STREAM("[grab_gear]: grab_bin1 error");
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
    arm1.RobotGoHome();
}