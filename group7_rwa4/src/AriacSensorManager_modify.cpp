#include "AriacSensorManager.h"
using namespace std;

AriacSensorManager::AriacSensorManager() :
    part_list {},
    part_pose_list {}
    // arm1 {"arm1"}
{
    camera_break_beam = sensor_nh_.advertise<osrf_gear::LogicalCameraImage>("current_parts", 1);

    orders_subscriber = sensor_nh_.subscribe("/ariac/orders", 10, 
        & AriacSensorManager::order_callback, this);

    break_beam_1_subscriber = sensor_nh_.subscribe("/ariac/break_beam_1_change", 10,
        & AriacSensorManager::break_beam_1_callback, this);

    break_beam_2_subscriber = sensor_nh_.subscribe("/ariac/break_beam_2_change", 10,
        & AriacSensorManager::break_beam_2_callback, this);

    init_ = false;
    cam_1_ = false;
    time_flag = false;
    order_number = 0;
}

AriacSensorManager::~AriacSensorManager() {}

void AriacSensorManager::order_callback(const osrf_gear::Order::ConstPtr & order_msg) {
    ROS_INFO_STREAM("Received order:\n" << *order_msg);
    received_orders_.push_back(*order_msg);
    setDesiredParts();
}


void AriacSensorManager::logical_camera_callback_1(
    const osrf_gear::LogicalCameraImage::ConstPtr& image_msg)
{
    if (image_msg->models.size() == 0){
        ROS_WARN_THROTTLE(5, "---Logical camera 1 does not detect things---");
        return;
    }
    ROS_INFO_STREAM("Logical camera captures '" << image_msg->models.size() << "' objects.");

    ros::Duration timeout(0.2);
}

void AriacSensorManager::camera_break_beam_cb() {
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    clock = ros::Time::now();

    for (auto & msg : image_msg->models){
       
        geometry_msgs::TransformStamped transformStamped;

        // find if this type of part has passed by before
        if (part_counter.find(msg.type) == part_counter.end())
            part_counter.insert(make_pair(msg.type, 0));
        else{
            // if ((ros::Time::now().toSec() - clock.sec) > 0.9)
                part_counter[msg.type]++;
        }
        string camera_frame = "logical_camera_1_" + msg.type + "_" + to_string(part_counter[msg.type]) + "_frame";
        ROS_INFO_STREAM("The frame is named: " << camera_frame);
        try{
            transformStamped = tfBuffer.lookupTransform("world", camera_frame, ros::Time(0), timeout);
            part_list.push_back(msg.type);
            geometry_msgs::Pose part_pose; 
            part_pose.position.x = transformStamped.transform.translation.x;
            part_pose.position.y = transformStamped.transform.translation.y;
            part_pose.position.z = transformStamped.transform.translation.z;
            part_pose.orientation.x = transformStamped.transform.rotation.x;
            part_pose.orientation.y = transformStamped.transform.rotation.y;
            part_pose.orientation.z = transformStamped.transform.rotation.z;
            part_pose.orientation.w = transformStamped.transform.rotation.w;

            // // Transform quaternion to rpy
            // tf2::Quaternion q(transformStamped.transform.rotation.x,
            //     transformStamped.transform.rotation.y,
            //     transformStamped.transform.rotation.z,
            //     transformStamped.transform.rotation.w);
            // tf2::Matrix3x3 m(q);
            // double roll, pitch, yaw;
            // m.getRPY(roll, pitch, yaw);
            // ROS_INFO("%s in world frame [%f,%f,%f] [%f,%f,%f] \n", camera_frame.c_str(), 
            //     transformStamped.transform.translation.x,
            //     transformStamped.transform.translation.y, 
            //     transformStamped.transform.translation.z,
            //     roll,
            //     pitch,
            //     yaw);
            part_pose_list[camera_frame].emplace_back(part_pose);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s\n",ex.what());
            part_counter[msg.type]--;
            // ROS_INFO_STREAM("Duration between the last and the current read: " << ros::Time::now() - clock);
            // ros::Duration(1.0).sleep();

            // continue;
        }
        break;
    }

}

void AriacSensorManager::break_beam_1_callback(const osrf_gear::Proximity::ConstPtr & msg) {
    /*
    Callback function working process:
    1. when a part fully goes through braek beam sensor, start counting.
    2. Once another part passes, check if the duration between this time and the next time 
        the beam be triggered is less than 1 sec:
            yes: consider the duration 
            no: keep counting
    */
    if (msg->object_detected){
        ROS_WARN("Break beam triggered");   
        camera_break_beam.publish();
    }
    else
        ROS_WARN("End of break beam triggered");
}

void AriacSensorManager::break_beam_2_callback(const osrf_gear::Proximity::ConstPtr & msg) {
    if (msg->object_detected){
        ROS_INFO_STREAM("Break beam triggered. The part is '" << part_list.front() << "'");
        auto element_itr = desired_parts.find(part_list.front());
        if (desired_parts.find(part_list.front()) != desired_parts.end()){
            ROS_INFO_STREAM("!!!!!!Pick this part!!!!");
            // bool if_pick = arm1.PickPart();
            // if (if_pick)
                // desired_parts.erase(element_itr);
        }
        else
            ROS_INFO_STREAM("Let it go ~~~~~~~~~");      
        part_list.pop_front();
    }
}


// void PickBaseOnOrder(){
//     part_list = sensor.get_part_list();
    

//     for (const auto & order : received_orders_){
//         auto order_id = order.order_id;
//         auto shipments = order.shipments;
//         for (const auto & shipment : shipments){
//             auto shipment_type = shipment.shipment_type;
//             auto products = shipment.products;
//             ROS_INFO_STREAM("Order ID: " << order_id);
//             ROS_INFO_STREAM("Shipment Type: " << shipment_type);
//             for (const auto & product: products){
//                 ros::spinOnce();
//                 product_type_pose_.first = product.type;
//                 //ROS_INFO_STREAM("Product type: " << product_type_pose_.first);
//                 product_type_pose_.second = product.pose;
//                 ROS_INFO_STREAM("Product pose: " << product_type_pose_.second.position.x);
//                 pick_n_place_success =  PickAndPlace(product_type_pose_);
//                 //--todo: What do we do if pick and place fails?
//             }
//             int finish=1;
//         }
//  }

void AriacSensorManager::setDesiredParts(){
    ROS_INFO_STREAM(">>>>>>>>> Setting desired parts");
    auto current_order = received_orders_[order_number];
    auto order_id = current_order.order_id;
    auto shipments = current_order.shipments;
    for (const auto &shipment: shipments){
        auto shipment_type = shipment.shipment_type;
        auto products = shipment.products;
        ROS_INFO_STREAM("Order ID: " << order_id);
        ROS_INFO_STREAM("Shipment Type: " << shipment_type);
        for (const auto &product: products)
            desired_parts.insert(product.type);
    }
    ROS_INFO_STREAM(">>>>>>>> The current desired_parts are:");
    for (const auto & part : desired_parts)
        std::cout << part << std::endl;
    if (!order_id.empty())
        ++order_number;
}