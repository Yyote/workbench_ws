#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>

ros::Publisher goal_point_pub;
tf2_ros::Buffer *p_tfBuffer;
tf2_ros::TransformListener *p_tfListener;


void changed_position_callback(const nav_msgs::Odometry::ConstPtr &catched_odom)
{
	geometry_msgs::PointStamped goal_point;

		geometry_msgs::TransformStamped transformStamped;
		try
		{
			transformStamped = p_tfBuffer->lookupTransform("nav_goal_frame", "base_footprint", ros::Time(0));
		}
		catch (tf2::TransformException &ex) 
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
		}

		goal_point.header.frame_id = "base_footprint";
		goal_point.header.stamp = ros::Time::now();
		goal_point.point.x = transformStamped.transform.translation.x;
		goal_point.point.y = transformStamped.transform.translation.y;
		goal_point.point.z = transformStamped.transform.translation.z;

		goal_point_pub.publish(goal_point);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "my_tf2_listener");

	ros::NodeHandle node;
	goal_point_pub = node.advertise<geometry_msgs::PointStamped>("/goal", 1000);
	ros::Subscriber odom_sub = node.subscribe("/odom", 1, &changed_position_callback);

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	// p_goal_point_pub = &goal_point_pub;
	p_tfBuffer = &tfBuffer;
	p_tfListener = &tfListener;

	// p_goal_point_pub = &goal_point_pub;

	ros::spin();
	return 0;


};