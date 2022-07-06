#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>


std::string child_frame_name;
std::string father_frame_name;


int main(int argc, char **argv)
{
    std::string node_name;
    node_name = argv[9];
    
    ros::init(argc,argv, node_name);
    
    if(argc != 10)
    {
        ROS_ERROR("Invalid number of parameters\nusage: static_turtle_tf2_broadcaster child_frame_name x y z roll pitch yaw");
        return -1;
    }
    
    if(strcmp(argv[1],"world")==0 && strcmp(argv[2],"world")==0)
    {
        ROS_ERROR("Your static frame name cannot be 'world'");
        return -1;
    }
    
    child_frame_name = argv[1];
    father_frame_name = argv[2];
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped static_transformStamped;

    static_transformStamped.header.stamp = ros::Time::now();
    static_transformStamped.header.frame_id = father_frame_name;
    static_transformStamped.child_frame_id = child_frame_name;
    static_transformStamped.transform.translation.x = atof(argv[3]);
    static_transformStamped.transform.translation.y = atof(argv[4]);
    static_transformStamped.transform.translation.z = atof(argv[5]);

    tf2::Quaternion quat;
    quat.setRPY(atof(argv[6]), atof(argv[7]), atof(argv[8]));
    
    static_transformStamped.transform.rotation.x = quat.x();
    static_transformStamped.transform.rotation.y = quat.y();
    static_transformStamped.transform.rotation.z = quat.z();
    static_transformStamped.transform.rotation.w = quat.w();
    
    static_broadcaster.sendTransform(static_transformStamped);
    
    ROS_INFO("Spinning until killed publishing %s to world", child_frame_name.c_str());
    ros::spin();
    
    return 0;
};