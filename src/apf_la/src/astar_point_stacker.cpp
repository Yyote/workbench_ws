// Create a node that takes point array from astar and stack them into a vector
#include "ros/ros.h"
#include "apf_la/GlobalTrajectory.h"

apf_la::GlobalTrajectory trajectory_storage;

void astar_cb(const apf_la::GlobalTrajectory::ConstPtr& trajectory)
{
    for (int i = 0; i < trajectory->waypoints.capacity(); i++)
    {
        // make a copy of the trajectory object in trajectory_storage
        trajectory_storage.waypoints.push_back(trajectory->waypoints[i]);
        trajectory_storage.waypoints.at(i).header.stamp = trajectory->waypoints.at(i).header.stamp;
        trajectory_storage.waypoints.at(i).header.frame_id = trajectory->waypoints.at(i).header.frame_id;
    }
}

void odom_cb(const nav_msgs::Odometry::ConstPtr& odom)
{
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "astar_point_stacker");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<>("/astar/trajectory", 1000, astar_cb);
    ros::Subscriber sub = n.subscribe<>("/odom", 1000, odom_cb);
    ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("/point_stacker/actual_goal", 1000);

    ros::spin();
    return 0;
}