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
    nav_msgs::Path path;

    tf2_ros::Buffer tf_buffer;
    
    geometry_msgs::TransformStamped trasf;

    ros::Time lastTime;
    double x,y,th;
    

public:
    Pub_sub_path_broad() {
        sub = n.subscribe("/amcl_pose", 1, &Pub_sub_path_broad::broadPath, this);
        path_pub = n.advertise<nav_msgs::Path>("/path",1);
    }

    void broadPath(const geometry_msgs::PoseWithCovarianceStamped &msg){

        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = msg.header.frame_id;
        pose.header.stamp = msg.header.stamp;
        pose.pose = msg.pose.pose;



        path.header.frame_id = "map";
        path.header.stamp = msg.header.stamp;
        path.poses.push_back(pose);

        path_pub.publish(path);

    }

};
int main(int argc, char **argv) {
    ros::init(argc, argv, "PathBroadcaster");
    Pub_sub_path_broad pubSubPathBroad;



    ros::spin();
    return 0;
}