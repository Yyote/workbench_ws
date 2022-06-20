// PotentialFields algorithm for lidar in PointCloud  !!2D!!
// 
// Listens to topic /transPC
// Publishes to topic /potential_field_result
// 
// frame: laser
// 
#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "beginner_tutorials/VelocitiesPC.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2/LinearMath/Quaternion.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

// PRESETUP 
bool publish_rviz_vizualization = 1;
int okcheck = 1;
float maxRange = 1; //max range to calculate

// SUB catchPC creation
ros::Subscriber catchPC_sub;

ros::Publisher vis_pub;
ros::Publisher speed_pub;

class EulerAngles {
    public:
    float roll;
    float pitch;
    float yaw;
};

double C = 0.01;


int shutdown()
{
    ROS_ERROR_STREAM(std::endl << std::endl << "terminated. code is " << okcheck);
    ros::shutdown();
    return okcheck;
}

void got_scanCallback(const sensor_msgs::PointCloud::ConstPtr& catchedCloud)
{
    // ROS_INFO_STREAM("Size of is --->" << catchedCloud->points.capacity() << std::endl);

    tf2::Quaternion q;
    EulerAngles angles;

        angles.roll = 0;
        angles.pitch = 0;
        angles.yaw = 0;

    visualization_msgs::Marker marker; // RVIZ marker init
    visualization_msgs::Marker marker2; // RVIZ marker init


    // CODEINFO
    // Calculate average x and y and make a vector of retraction from them
    int sumcount = 0;

    double finvec_x = 0;
    double finvec_y = 0;


    for(int i = 0; i < catchedCloud->points.capacity(); i++)
    {
        if(i % 3 == 0)
        {
            if(!(sqrt(pow(catchedCloud->points.at(i).x, 2) + pow(catchedCloud->points.at(i).y, 2)) >= maxRange || sqrt(pow(catchedCloud->points.at(i).x, 2) + pow(catchedCloud->points.at(i).y, 2)) <= 0.05 || catchedCloud->points.at(i).x < 0.001 && catchedCloud->points.at(i).x > -0.001 || catchedCloud->points.at(i).y < 0.001 && catchedCloud->points.at(i).y > -0.001))  
            {
                finvec_y -= C * catchedCloud->points.at(i).y / ((-1 + ((!catchedCloud->points.at(i).y < 0)*2)) * (catchedCloud->points.at(i).y) * (catchedCloud->points.at(i).y)); // ((-1 + ((!catchedCloud->points.at(i).y < 0)*2)) * 
                finvec_x -= C * catchedCloud->points.at(i).x / ((-1 + ((!catchedCloud->points.at(i).x < 0)*2)) * (catchedCloud->points.at(i).x) * (catchedCloud->points.at(i).x)); // ((-1 + ((!catchedCloud->points.at(i).x < 0)*2)) * 

                if(finvec_x > sqrt(maxRange)) //LOG NEW changed maxRange to sqrt(maxRange)
                {
                    finvec_x -= sqrt(maxRange);
                }
                else if(finvec_x < -sqrt(maxRange))
                {
                    finvec_x -= -sqrt(maxRange);
                }

                if (finvec_y > sqrt(maxRange))
                {
                    finvec_y -= sqrt(maxRange);
                }
                else if(finvec_y < -sqrt(maxRange))
                {
                    finvec_y -= -sqrt(maxRange);
                }

                sumcount += 1;
            }

            double bufx, bufy; //DEBUG
            bufx = finvec_x;
            bufy = finvec_y;          

            // DEBUG rinfo
            if(1)
            {
                ROS_INFO_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl << "FUNCTION NAME: average coordinates calculation" << std::endl 
                << "VARIABLES: list of vars" << "\ncloud capacity, finvec_x and y, vector length, i, sumcount:" << std::endl 
                << catchedCloud->points.capacity() << " "  << finvec_x << " " << finvec_y << " " << sqrt(pow(finvec_x, 2) + pow(finvec_y, 2)) << " " << i << " " << sumcount << std::endl 
                << "catchedCloud->points.at(i).x --> " <<  std::endl << catchedCloud->points.at(i).x << "catchedCloud->points.at(i).y --> " << catchedCloud->points.at(i).y << std::endl 
                <<"_________________________________" << std::endl << "_________________________________" << std::endl);
            }

            // DEBUG rinfo
            if (finvec_x == finvec_y && finvec_x != 0)
            {
                ROS_ERROR_STREAM("FINVECX IS EQ TO FINVECY. terminated.");
                shutdown();
            }

            if(finvec_x - bufx > 50 || finvec_y - bufy > 50) //DEBUG
            {
                ROS_ERROR_STREAM("FINVEC IS EXCEEDING LIMITS AND AVOIDS FILTRATION --> " << std::endl << "finvec_x ---> " << finvec_x << std::endl << "finvec_y ---> " << finvec_y);
                okcheck = -1;
                shutdown();
            }
        }
    }
    ROS_WARN_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl << "FUNCTION NAME: result vector in variables" << std::endl << "VARIABLES: list of vars:" << std::endl << "finvec_x -->" << finvec_x << std::endl << "finvec_y -->" << finvec_y << std::endl << "vector length -->" << sqrt(pow(finvec_x , 2) + pow(finvec_y , 2)) <<  std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl);


    //IDEA make a bullshit filter for potential fields and create max passed speed constant

    // CODEINFO angle calculation to pass to rviz markers ---> calculate current(i) vector's angle 
    angles.yaw = atan(finvec_y/finvec_x);
    q.setRPY(angles.roll, angles.pitch, angles.yaw);
    q = q.normalize(); //SOLVED something is going with angle. cant say what, maybe its not a bug ---> in velocity calculation there was an if statement that tried to pass only > 0.001 coordiantes. It totally deleted all negative values from coordinates


    // DEBUG rinfo
    if(0)
    {
        ROS_INFO_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl 
        << "FUNCTION NAME: angle calc" << std::endl 
        << "VARIABLES: " << std::endl 
        << "angles.yaw (atan)-->" << angles.yaw << std::endl << "finvec_y/finvec_x --> " << finvec_y/finvec_x << std::endl << "finvec_y --> " << finvec_y << std::endl << "finvec_x --> " << finvec_x << std::endl << std::endl 
        << "_________________________________" << std::endl << "_________________________________" << std::endl);
    }

    //*-*-*-*-*-*-*-*--*-*-*-*-*--*--*-*-*-*-*-*-*--*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*
    //------------------------------------------------------------------------------
    // VISUALIZATION //RVIZ visualiztion code // TODO make it a func
    //------------------------------------------------------------------------------
    marker.header.stamp = ros::Time::now();
    marker.header.frame_id = "laser"; //FRAME
    marker.ns = "speeds_namespace";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = sqrt(pow(finvec_x, 2) + pow(finvec_y, 2));
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;

    if(publish_rviz_vizualization == 1)
    {
        vis_pub.publish(marker); // PUB //RVIZ publish
    }

        marker.header.stamp = ros::Time::now();
    marker2.header.frame_id = "laser"; //FRAME
    marker2.ns = "speeds_namespace";
    marker2.id = 2;
    marker2.type = visualization_msgs::Marker::CYLINDER;
    marker2.action = visualization_msgs::Marker::ADD;
    marker2.pose.position.x = 0.0;
    marker2.pose.position.y = 0.0;
    marker2.pose.position.z = -0.05;
    marker2.pose.orientation.x = q.x();
    marker2.pose.orientation.y = q.y();
    marker2.pose.orientation.z = q.z();
    marker2.pose.orientation.w = q.w();
    marker2.scale.x = sqrt(maxRange);
    marker2.scale.y = sqrt(maxRange);
    marker2.scale.z = 0.05;
    marker2.color.a = 0.3;
    marker2.color.r = 0.0;
    marker2.color.g = 0.0;
    marker2.color.b = 1.0;

    if(publish_rviz_vizualization == 1)
    {
        vis_pub.publish(marker2); // PUB //RVIZ publish
    }
    //*-*-*-*-*-*-*-*--*-*-*-*-*--*--*-*-*-*-*-*-*--*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*


    //CODEINFO Twist init and pub
    geometry_msgs::Twist twist;
    if(abs(angles.yaw) - 0.2 < 0)
    {
        twist.angular.z = 0;
        if(sqrt(pow(finvec_x, 2) + pow(finvec_y, 2)) < 0.001)
        {
            twist.linear.x = 0;
            twist.linear.y = 0;
            ROS_ERROR_STREAM(std::endl << "Speed vector is too small" << std::endl);
        }
        else 
        {
            twist.linear.x = finvec_x;
            twist.linear.y = finvec_y;
        }
    }
    else if (angles.yaw > 0 && angles.yaw < 3.14 / 2)
    {
        ROS_ERROR_STREAM(std::endl << "Angle is correlating badly" << std::endl << "tan --> " << finvec_y/finvec_x << std::endl << "yaw --> " << angles.yaw * 180 / 3.14 << " degrees." << std::endl);
        twist.angular.z = 0.4;
    }
        else if (angles.yaw < 0 && angles.yaw > - 3.14 / 2)
    {
        ROS_ERROR_STREAM(std::endl << "Angle is correlating badly" << std::endl << "tan --> " << finvec_y/finvec_x << std::endl << "yaw --> " << angles.yaw * 180 / 3.14 << " degrees." << std::endl);
        twist.angular.z = -0.4;
    }
    
    speed_pub.publish(twist);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "potential_fields");


// NODEHANDLE
ros::NodeHandle cs;
ros::NodeHandle vlcts; //TODO check for unused nhs
ros::NodeHandle rvizor; // mrviz markers nh
ros::NodeHandle vlcts_in_PC;


    //PUB adverts
    vis_pub = rvizor.advertise<visualization_msgs::Marker>("/potential_field_result", 1000);
    speed_pub = vlcts.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    catchPC_sub = cs.subscribe("/transPC", 1000, got_scanCallback); // SUB catchPC sub


    ros::spin();

    return 0;
}

