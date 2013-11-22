#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <strands_datacentre/SetParam.h>

#include <math.h>

using namespace std;
int save;

ros::Publisher mileage_pub;
std_msgs::Float64 total_distance;
geometry_msgs::Point last_point;
ros::ServiceClient client;
strands_datacentre::SetParam srv;
int save_interval;

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

    if(save % save_interval == 0) {
        ros::param::set("/saved_mileage",total_distance.data);
        srv.request.param = "saved_mileage";
        if (client.call(srv))
            ROS_DEBUG("Save mileage: success");
        else
            ROS_WARN("Save mileage: failed");
        save = 0;
    }
    save++;
}

int main(int argc, char **argv)
{
    // Set up ROS.
    ros::init(argc, argv, "odom_mileage");
    ros::NodeHandle n;

    last_point.x = 0;
    last_point.y = 0;
    last_point.z = 0;

    save = 1;

    // Declare variables that can be modified by launch file or command line.
    string mileage_topic;
    string odom_topic;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.
    ros::NodeHandle private_node_handle_("~");
    private_node_handle_.param("mileage_topic", mileage_topic, string("/odom_mileage"));
    private_node_handle_.param("odom_topic", odom_topic, string("/odom"));
    private_node_handle_.param("save_interval", save_interval, 500);
    n.param("/saved_mileage", total_distance.data, 0.0);

    client = n.serviceClient<strands_datacentre::SetParam>("/config_manager/save_param");

    //Create a subscriber
    ros::Subscriber odom_sub = n.subscribe(odom_topic.c_str(), 50, &callback);

    // Create a publisher
    mileage_pub = n.advertise<std_msgs::Float64>(mileage_topic.c_str(), 10);

    ros::spin();
    return 0;
}
