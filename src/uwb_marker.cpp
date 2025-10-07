#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <uwb_uvdar_fusion/UwbUvdarResult.h>

ros::Publisher marker_pub;

void uwbCallback(const uwb_uvdar_fusion::UwbUvdarResult::ConstPtr& msg)
{
    static int counter = 0;
    visualization_msgs::Marker marker;
    
    // Header
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = msg->header.frame_id.empty() ? "uav2/odom" : msg->header.frame_id;
    
    // Basic info
    marker.ns = "uwb_points";
    marker.id = counter++; 
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    
    // Pose
    marker.pose.position.x = (msg->z) / 1000.0;
    marker.pose.position.y = -(msg->x) / 1000.0;
    marker.pose.position.z = -(msg->y) / 1000.0;
    marker.pose.orientation.w = 1.0; // no rotation
    
    // Scale
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    
    // Color based on ID
    if (msg->id == 28) {
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0; // red
    } else if (msg->id == 29) {
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0; // green
    } else {
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0; // default: white
    }
    marker.color.a = 0.8;
    
    // Lifetime 0 = forever
    marker.lifetime = ros::Duration(0);

    marker_pub.publish(marker);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "uwb_marker_node");
    ros::NodeHandle nh("~");

    // Publisher for markers
    marker_pub = nh.advertise<visualization_msgs::Marker>("/uwb_uwdar_markers", 100);

    // Subscriber to your existing topic
    ros::Subscriber sub = nh.subscribe("/uwb_uvdar_fusion_node/uwb_uvdar_result_raw", 10, uwbCallback);

    ROS_INFO("UWB marker publisher started...");
    ros::spin();

    return 0;
}
