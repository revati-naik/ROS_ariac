#include <algorithm>
#include <vector>
#include <deque>
#include <ros/ros.h>
#include <string>
#include <osrf_gear/Order.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <trajectory_msgs/JointTrajectory.h>

class MyCompetitionClass
{
private:
    // Subscriber
    ros::Subscriber orders_subscriber;
    // Order/Products
    std::vector<osrf_gear::Order> received_orders_;
    
public:
    explicit MyCompetitionClass(ros::NodeHandle &);

    /// Called when a new Order message is received.
    void order_callback(const osrf_gear::Order::ConstPtr &);
};