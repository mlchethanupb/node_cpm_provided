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
#include <pdk_ros_msg/PerceivedObjectContainer.h>
#include <vector>
#include <cmath>


// Structure to hold the Euler angles
struct EulerAngles {
    double roll;   // Rotation around x-axis (in radians)
    double pitch;  // Rotation around y-axis (in radians)
    double yaw;    // Rotation around z-axis (in radians)
};

ros::Publisher  pub_cpm_provided;
float latitude;
float longitude;
float altitude;
float heading;
float orientation_angle;
EulerAngles euler_angle;
std::vector<etsi_its_msgs::PerceivedObject> etsi_prcvd_obj_cntnr;
uint32_t number_of_objects;

// Function to convert a quaternion to Euler angles
EulerAngles quaternionToEulerAngles(double qx, double qy, double qz, double qw) {
    EulerAngles euler;

    // Roll (rotation around x-axis)
    euler.roll = atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy));

    // Pitch (rotation around y-axis)
    double sinp = 2 * (qw * qy - qz * qx);
    if (fabs(sinp) >= 1)
        euler.pitch = copysign(M_PI / 2, sinp);  // Use 90 degrees if out of range
    else
        euler.pitch = asin(sinp);

    // Yaw (rotation around z-axis)
    euler.yaw = atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz));

    return euler;
}

void heading_Callback(const geometry_msgs::QuaternionStamped::ConstPtr& msg)
{
    // Process the received message here
    // Access message fields using msg->field_name

    ROS_INFO("Received Quaternion: [%f, %f, %f, %f]", msg->quaternion.x, msg->quaternion.y, msg->quaternion.z, msg->quaternion.w);
    euler_angle = quaternionToEulerAngles(msg->quaternion.x, msg->quaternion.y, msg->quaternion.z, msg->quaternion.w);
}


void gps_filetered_Callback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
  // Access the fields of the received message
  // Example:
  ROS_INFO("Received GPS Fix: Latitude: %f, Longitude: %f, Altitude: %f", msg->latitude, msg->longitude, msg->altitude);

  latitude = msg->latitude;
  longitude = msg->longitude;
  altitude = msg->altitude;
}

void get_prcvd_objs(const pdk_ros_msg::PerceivedObjectContainer::ConstPtr &rosPerceivedObjectContainer ){

    std::cout << "object list container received: " << std::endl;
    number_of_objects = rosPerceivedObjectContainer->numberOfPerceivedObjects;
    etsi_prcvd_obj_cntnr.clear();
    auto prcvd_objs = rosPerceivedObjectContainer->perceivedObjects;



    for (auto obj: prcvd_objs){

        etsi_its_msgs::PerceivedObject rosObj;

        rosObj.objectID = obj.objectID;
        rosObj.timeOfMeasurement = obj.measurementDeltaTime;
        rosObj.objectAge = obj.objectAge;

        rosObj.objectConfidence = obj.objectPerceptionQuality;
        rosObj.xDistance.value = obj.objectDimensionX.value;
        rosObj.xDistance.confidence = obj.objectDimensionX.confidence;
        rosObj.yDistance.value = obj.objectDimensionY.value;
        rosObj.yDistance.confidence = obj.objectDimensionY.confidence;
        rosObj.zDistance.value = obj.objectDimensionZ.value;
        rosObj.zDistance.confidence = obj.objectDimensionZ.confidence;

        rosObj.xSpeed.value  = obj.velocity.cartesianVelocity.xVelocity.vel_comp_value;
        rosObj.ySpeed.value  = obj.velocity.cartesianVelocity.yVelocity.vel_comp_value;
        rosObj.xAcceleration.value = obj.acceleration.cartesianAcceleration.xAcceleration.value;
        rosObj.yAcceleration.value = obj.acceleration.cartesianAcceleration.yAcceleration.value;
        rosObj.objectAge = obj.objectAge;
        rosObj.objectAge = obj.objectAge;
        rosObj.objectAge = obj.objectAge;
        rosObj.objectAge = obj.objectAge;
        rosObj.objectAge = obj.objectAge;

        etsi_prcvd_obj_cntnr.push_back(rosObj);
    }


}


void construct_CPM_provided(){
    etsi_its_msgs::CPM cpm;

    cpm.its_header.protocol_version = 2;
    cpm.its_header.station_id = 420;

    cpm.generation_delta_time = 23234;

    cpm.station_type.value = 2;

    int64_t conversion_factor = 1e7; // Adjust this factor based on your desired precision

    // Convert latitude and longitude to int64
    int64_t latitude_int = static_cast<int64_t>(latitude * conversion_factor);
    int64_t longitude_int = static_cast<int64_t>(longitude * conversion_factor);
    int64_t altitude_int = static_cast<int64_t>(altitude * 1e3);


    //reference position
    cpm.reference_position.latitude = latitude_int;
    cpm.reference_position.longitude = longitude_int;

    cpm.reference_position.position_confidence.semi_major_confidence = 1;
    cpm.reference_position.position_confidence.semi_minor_confidence = 2;
    cpm.reference_position.position_confidence.semi_major_orientation = 3;

    cpm.reference_position.altitude.value = altitude_int;
    cpm.reference_position.altitude.confidence = 100;

    //originating vehicle container 
    cpm.originatingVehicleContainer.heading.value = euler_angle.roll;
    cpm.originatingVehicleContainer.heading.confidence = 100;

    cpm.originatingVehicleContainer.speed.value = euler_angle.pitch;
    cpm.originatingVehicleContainer.speed.confidence = 100;

    cpm.originatingVehicleContainer.vehicleOrientationAngle.value = euler_angle.yaw;
    cpm.originatingVehicleContainer.vehicleOrientationAngle.value = 100;

    cpm.has_sensor_information_container = false;
    cpm.has_list_of_perceived_object = true;

    cpm.listOfPerceivedObjects.perceivedObjectContainer = std::vector<etsi_its_msgs::PerceivedObject>();
    //for(auto obj: etsi_prcvd_obj_cntnr ){
    //   cpm->listOfPerceivedObjects.perceivedObjectContainer.push_back(obj); 
   // }
    cpm.numberOfPerceivedObjects = number_of_objects;

    pub_cpm_provided.publish(cpm);
    
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
    ros::Subscriber prcvd_objs = nh.subscribe("car2x_unit", 10, &get_prcvd_objs);
    pub_cpm_provided = nh.advertise<etsi_its_msgs::CPM>("cpm_provided", 20);

    while(ros::ok() == true){
        construct_CPM_provided();
        sleep(0.1);
    }

    // ROS main loop
    ros::spin();

    return 0;
}