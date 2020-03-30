#include "MyCompetitionClass.h"

MyCompetitionClass::MyCompetitionClass(ros::NodeHandle & nh)
{
    orders_subscriber = nh.subscribe("/ariac/orders", 10,
        & MyCompetitionClass::order_callback, this);
}

void MyCompetitionClass::order_callback(const osrf_gear::Order::ConstPtr & order_msg) 
{
    ROS_INFO_STREAM("Received order:\n" << *order_msg);
    received_orders_.push_back(*order_msg);
}