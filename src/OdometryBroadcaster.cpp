#include <ros/ros.h>
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class Pub_sub_path_broad{

private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher path_pub;
    ros::Publisher pose_pub;
    tf2_ros::TransformBroadcaster odom_broadcaster;

    ros::Time lastTime;
    double x,y,th;
    

public:
    Pub_sub_path_broad() {
        sub = n.subscribe("/odom", 1, &Pub_sub_path_broad::broadOdom, this);
    }

    void broadOdom(const nav_msgs::Odometry &msg){
        
        ros::Time currentTime = msg.header.stamp;
        geometry_msgs::TransformStamped odometryTransformation;



        odometryTransformation.header.stamp = currentTime;
        odometryTransformation.header.frame_id = "odom";
        odometryTransformation.child_frame_id = "base_footprint";
        odometryTransformation.transform.translation.x = msg.pose.pose.position.x;
        odometryTransformation.transform.translation.y = msg.pose.pose.position.y;
        odometryTransformation.transform.translation.z = msg.pose.pose.position.z;
        

        odometryTransformation.transform.rotation.x = msg.pose.pose.orientation.x;
        odometryTransformation.transform.rotation.y = msg.pose.pose.orientation.y;
        odometryTransformation.transform.rotation.z = msg.pose.pose.orientation.z;
        odometryTransformation.transform.rotation.w = msg.pose.pose.orientation.w;

        odom_broadcaster.sendTransform(odometryTransformation);
    }

};
int main(int argc, char **argv) {
    ros::init(argc, argv, "OdometryBroadcaster");
    Pub_sub_path_broad pubSubOdometryBroad;



    ros::spin();
    return 0;
}