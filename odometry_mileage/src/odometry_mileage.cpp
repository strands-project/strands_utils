#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>

#include <math.h>

using namespace std;

ros::Publisher mileage_pub;
std_msgs::Float64 total_distance;
geometry_msgs::Point last_point;

void callback(const nav_msgs::Odometry::ConstPtr &odom)
{
    if(last_point.x == 0 && last_point.y == 0 && last_point.z == 0) {
        last_point = odom->pose.pose.position;
        return;
    }

    double distance = sqrt(pow(odom->pose.pose.position.x-last_point.x,2)+pow(odom->pose.pose.position.y-last_point.y,2));
    total_distance.data += distance;

    mileage_pub.publish(total_distance);

    last_point = odom->pose.pose.position;

}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "odom_mileage");
    ros::NodeHandle n;

    last_point.x = 0;
    last_point.y = 0;
    last_point.z = 0;

    total_distance.data = 0;

    // Declare variables that can be modified by launch file or command line.
    string mileage_topic;
    string odom_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("mileage_topic", mileage_topic, string("/odom_mileage"));
    private_node_handle_.param("odom_topic", odom_topic, string("/odom"));

    ros::Subscriber odom_sub = n.subscribe(odom_topic.c_str(), 10, &callback);

    // Create a publisher
    mileage_pub = n.advertise<std_msgs::Float64>(mileage_topic.c_str(), 10);

    ros::spin();
    return 0;
}
