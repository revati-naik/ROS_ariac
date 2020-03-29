#include "MyCompetitionClass.h"
#include "AriacSensorManager.h"
// #include "robot_controller.h"
#include <string>

int main(int argc, char ** argv) {
    // Last argument is the default name of the node.
    std::string node_name {"sensor_node"};
    ros::init(argc, argv, node_name);
    ros::AsyncSpinner spinner(0);

    AriacSensorManager sensor;
    ROS_INFO_STREAM(">>>>>>>>>>>Sensor node activate");

    spinner.start();
    ros::waitForShutdown();
}
