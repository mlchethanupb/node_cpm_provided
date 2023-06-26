#include <ros/ros.h>
#include <sys/socket.h>
#include <ctime>
#include <cstring>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <std_msgs/String.h>
#include <etsi_its_msgs/CPM.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <vector>




void heading_Callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    // Process the received message here
    // Access message fields using msg->field_name

    ROS_INFO("Received Quaternion: [%f, %f, %f, %f]", msg->quaternion.x, msg->quaternion.y, msg->quaternion.z, msg->quaternion.w);
}


void gps_filetered_Callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  // Access the fields of the received message
  // Example:
  ROS_INFO("Received GPS Fix: Latitude: %f, Longitude: %f, Altitude: %f", msg->latitude, msg->longitude, msg->altitude);
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "node_cpm_provided");

    // Create a ROS node handle
    ros::NodeHandle nh;

    // Create a ROS subscriber
    ros::Subscriber sub_gps_filtered = nh.subscribe("/gps/filtered", 10, &gps_filetered_Callback);
    ros::Subscriber sub_heading = nh.subscribe("heading", 10, &heading_Callback);


    // ROS main loop
    ros::spin();

    return 0;
}