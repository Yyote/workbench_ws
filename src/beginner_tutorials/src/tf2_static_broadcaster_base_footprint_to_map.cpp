// #include <ros/ros.h>
// #include <tf2_ros/static_transform_broadcaster.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <tf2/LinearMath/Quaternion.h>

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "base_footprint_to_map_tf2");
//     static tf2_ros::StaticTransformBroadcaster static_broadcaster;
//     geometry_msgs::TransformStamped static_transformStamped;

//     static_transformStamped.header.stamp = ros::Time::now();
//     static_transformStamped.header.frame_id = "map";
//     static_transformStamped.child_frame_id = "odom";
//     static_transformStamped.transform.translation.x = 0;
//     static_transformStamped.transform.translation.y = 0;
//     static_transformStamped.transform.translation.z = 0;
//     tf2::Quaternion quat;
//     static_transformStamped.transform.rotation.x = 0;
//     static_transformStamped.transform.rotation.y = 0;
//     static_transformStamped.transform.rotation.z = 0;
//     static_transformStamped.transform.rotation.w = 1;
//     static_broadcaster.sendTransform(static_transformStamped);
//     ROS_INFO_STREAM("Spinning until killed publishing" << "base_footprint" <<  " to map");
//     ros::spin();
//     return 0;
// }

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/tf.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

void poseCallback(const nav_msgs::OdometryPtr& odom){
    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "odom";
    transformStamped.transform.translation.x = odom->pose.pose.position.x;
    transformStamped.transform.translation.y = odom->pose.pose.position.y;
    transformStamped.transform.translation.z = odom->pose.pose.position.z;   

    tf2::Quaternion q;
    q.setX(odom->pose.pose.orientation.x);
    q.setY(odom->pose.pose.orientation.y);
    q.setZ(odom->pose.pose.orientation.z);
    q.setW(-odom->pose.pose.orientation.w);  

    // tf2::Quaternion qworld;
    // qworld.setX(0);
    // qworld.setY(0);
    // qworld.setZ(0);
    // qworld.setW(1);    

    // tf2::Quaternion qrotation;
    // qrotation = qworld * q;   

    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.w = q.w();
    transformStamped.transform.rotation.z = q.z();

    br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "odom_to_map");    
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/odom", 10, &poseCallback);

    ros::spin();
    return 0;
};

