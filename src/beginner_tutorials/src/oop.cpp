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
#include "visualization_msgs/MarkerArray.h"
#include "geometry_msgs/Twist.h"
#include <cmath>


class MarkerHandler {
    private:
    visualization_msgs::Marker marker;
    ros::NodeHandle rvizor;
    ros::Publisher marker_pub;

    public:
    MarkerHandler()
    {
        marker.header.stamp = ros::Time::now();
    }

    MarkerHandler(std::string pub_topic) 
    {
        marker.header.stamp = ros::Time::now();
        marker_pub = rvizor.advertise<visualization_msgs::Marker>(pub_topic, 1000);
    }

    void advertise_to_topic(std::string pub_topic)
    {
        marker_pub = rvizor.advertise<visualization_msgs::Marker>(pub_topic, 1000);
    }

    void set_frame(std::string frame_id)
    {
        marker.header.frame_id = frame_id;
    }

    void set_namespace(std::string namespace_str)
    {
        marker.ns = namespace_str;
    }

    void set_id(int id)
    {
        marker.id = id;
    }

    void set_type(int enum_number)
    {
        /*
            ARROW = 0u,
            CUBE = 1u,
            SPHERE = 2u,
            CYLINDER = 3u,
            LINE_STRIP = 4u,
            LINE_LIST = 5u,
            CUBE_LIST = 6u,
            SPHERE_LIST = 7u,
            POINTS = 8u,
            TEXT_VIEW_FACING = 9u,
            MESH_RESOURCE = 10u,
            TRIANGLE_LIST = 11u,
        */
        if (enum_number == 0)
        {
            marker.type = visualization_msgs::Marker::ARROW;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 1)
        {
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 2)
        {
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 3)
        {
            marker.type = visualization_msgs::Marker::CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 4)
        {
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 5)
        {
            marker.type = visualization_msgs::Marker::LINE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 6)
        {
            marker.type = visualization_msgs::Marker::CUBE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 7)
        {
            marker.type = visualization_msgs::Marker::SPHERE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 8)
        {
            marker.type = visualization_msgs::Marker::POINTS;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 9)
        {
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 10)
        {
            marker.type = visualization_msgs::Marker::MESH_RESOURCE;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else if (enum_number == 11)
        {
            marker.type = visualization_msgs::Marker::TRIANGLE_LIST;
            marker.action = visualization_msgs::Marker::ADD;
        } 

        else ROS_ERROR_STREAM("NO SUCH POSITION IN THE LIST" << std::endl);
    }

    void set_pose(double x, double y, double z, tf2::Quaternion q)
    {
        marker.pose.position.x = x;
        marker.pose.position.y = y;
        marker.pose.position.z = z;
        marker.pose.orientation.x = q.getX();
        marker.pose.orientation.y = q.getY();
        marker.pose.orientation.z = q.getZ();
        marker.pose.orientation.w = q.getW();
    }

    void set_scale(float x, float y, float z)
    {
        marker.scale.x = x;
        marker.scale.y = y;
        marker.scale.z = z;
    }

    void set_color(float alpha, float red, float green, float blue)
    {
        marker.color.a = alpha;
        marker.color.r = red;
        marker.color.g = green;
        marker.color.b = blue;
    }

    void publish_marker()
    {
        marker_pub.publish(marker);
    }
};

class EulerAngles {
    public:
    float roll;
    float pitch;
    float yaw;
};

class PotentialFieldRepulsive {
    private:
    double C;
    float maxRange; //max range to calculate
    float minRange; //min range to calculate
    std::string final_vel_arrow_topic;
    EulerAngles angles;
    MarkerHandler final_vel_arrow;
    ros::Publisher vis_arr_pub;
    ros::NodeHandle rvizor;
    int publish_rviz_vizualization = 1;

    int shutdown()
    {
        ROS_ERROR_STREAM(std::endl << std::endl << "terminated." << std::endl);
        ros::shutdown();
        return -1;
    }

    public:
    double finvec_x;
    double finvec_y;

    PotentialFieldRepulsive() 
    {
        C = 0.001;
        minRange = 0.2;
        maxRange = 1;
        finvec_x = 0;
        finvec_y = 0;
        angles.roll = 0;
        angles.pitch = 0;
        angles.yaw = 0;     
        final_vel_arrow_topic = "/potential_field_result";
        final_vel_arrow.advertise_to_topic(final_vel_arrow_topic);
        vis_arr_pub = rvizor.advertise<sensor_msgs::PointCloud>("/debug_points", 1000);
    }   

    void calculate_repulse_vector(const sensor_msgs::PointCloud::ConstPtr& catchedCloud)
    {
        sensor_msgs::PointCloud debug_cloud; // LOG NEW debug_cloud

        debug_cloud.points.resize(catchedCloud->points.capacity()); // LOG NEW debug cloud
        debug_cloud.header.stamp = ros::Time::now();
        debug_cloud.header.frame_id = "laser"; //FRAME


        // CODEINFO
        // Calculate average x and y and make a vector of retraction from them
        for(int i = 0; i < catchedCloud->points.capacity(); i++)
        {
            //************************    PSEUDO CONSTS     **************************
            double vector_length = sqrt( pow(catchedCloud->points.at(i).x, 2) + pow(catchedCloud->points.at(i).y, 2) );
            //************************************************************************

            //************************    LOCAL VARS     *****************************
            double tmpvec_x, tmpvec_y;
            //************************************************************************

            debug_cloud.points.at(i).x = NULL;
            debug_cloud.points.at(i).y = NULL;
            debug_cloud.points.at(i).z = NULL;

            int point_step = 1; // LOG NEW added point_step variable to skip points in such way that it won't effect the amplitude of the resulting vector

            if(i % point_step == 0)
            {
                if(!( pow(catchedCloud->points.at(i).x, 2) + pow(catchedCloud->points.at(i).y, 2) >= maxRange * maxRange || pow(catchedCloud->points.at(i).x, 2) + pow(catchedCloud->points.at(i).y, 2) <= minRange * minRange))  // Проверка на попадание в зону видимости
                {
                    //DEBUG INFO rinfo
                    //****************************************************************************************************
                    if(0)
                    {
                        ROS_INFO_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl 
                        << "FUNCTION NAME: zero fight" << std::endl 
                        << "VARIABLES: "
                        << "vector length -->"  << sqrt(pow(catchedCloud->points.at(i).x, 2) + pow(catchedCloud->points.at(i).y, 2)) << std::endl
                        << "maxRange -->"  << maxRange << std::endl 
                        << "_________________________________" << std::endl << "_________________________________" << std::endl);
                    }
                    //****************************************************************************************************

                    if (catchedCloud->points.at(i).x != 0)
                    {
                        tmpvec_y = - C * point_step * ( (1 / vector_length) * catchedCloud->points.at(i).y / vector_length)/* - (maxRange / sqrt(2))*/ ; // BUG check atan limits and adjust the formula
                        tmpvec_x = - C * point_step * ( (1 / vector_length) * catchedCloud->points.at(i).x / vector_length)/* - (maxRange / sqrt(2))*/; // SOLVED change the formula for the pithagorean
                    }
                    else
                    {
                        tmpvec_y = C * point_step * ( (1 / vector_length) * sin(atan(catchedCloud->points.at(i).x / catchedCloud->points.at(i).y))/* - (maxRange / sqrt(2))*/ ); // BUG check atan limits and adjust the formula
                        tmpvec_x = C * point_step * ( (1 / vector_length) * cos(atan(catchedCloud->points.at(i).x / catchedCloud->points.at(i).y))/* - (maxRange / sqrt(2))*/ ); // SOLVED change the formula for the pithagorean
                    }

                    //DEBUG INFO rwarn
                    //****************************************************************************************************
                    if(1)
                    {
                        ROS_WARN_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl 
                        << "FUNCTION NAME: tmpvec_y check" << std::endl 
                        << "VARIABLES: " << std::endl
                        << "catched point y -->"  << catchedCloud->points.at(i).y << std::endl 
                        << "catched point x -->"  << catchedCloud->points.at(i).x << std::endl 
                        << "tmpvec_y -->"  << tmpvec_y << std::endl 
                        << "sin(atan(catchedCloud->points.at(i).y / catchedCloud->points.at(i).x)) * (1 / vec_length) -->"  << (1 / vector_length) * sin(atan(catchedCloud->points.at(i).y / catchedCloud->points.at(i).x)) << std::endl 
                        << "tmpvec_x -->"  << tmpvec_x << std::endl 
                        << "cos(atan(catchedCloud->points.at(i).y / catchedCloud->points.at(i).x)) * (1 / vec_length) -->"  << (1 / vector_length) * cos(atan(catchedCloud->points.at(i).y / catchedCloud->points.at(i).x)) << std::endl 
                        << "_________________________________" << std::endl << "_________________________________" << std::endl);
                    }
                    //****************************************************************************************************

                    if(catchedCloud->points.at(i).x > 0 && tmpvec_x > 0)
                    {
                        tmpvec_x = -tmpvec_x;
                    }

                    finvec_x += tmpvec_x;
                    finvec_y += tmpvec_y;

                    debug_cloud.points.at(i).x = catchedCloud->points.at(i).x;
                    debug_cloud.points.at(i).y = catchedCloud->points.at(i).y; // LOG NEW debug_cloud
                    debug_cloud.points.at(i).z = catchedCloud->points.at(i).z;
                }

                if(publish_rviz_vizualization == 1)
                {
                    vis_arr_pub.publish(debug_cloud); //PUB vis_arr debug_cloud
                }

                double bufx, bufy; //DEBUG rerror related
                bufx = finvec_x;
                bufy = finvec_y;          

                // DEBUG INFO rinfo
                if(0)
                {
                    ROS_INFO_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl << "FUNCTION NAME: average coordinates calculation" << std::endl 
                    << "VARIABLES: list of vars" << "\ncloud capacity, finvec_x and y, vector length, i:" << std::endl 
                    << catchedCloud->points.capacity() << " "  << finvec_x << " " << finvec_y << " " << sqrt(pow(finvec_x, 2) + pow(finvec_y, 2)) << " " << i << std::endl 
                    << "catchedCloud->points.at(i).x --> " << catchedCloud->points.at(i).x << std::endl << "catchedCloud->points.at(i).y --> " << catchedCloud->points.at(i).y << std::endl 
                    <<"_________________________________" << std::endl << "_________________________________" << std::endl);
                }

                // DEBUG rerror
                if (finvec_x == finvec_y && finvec_x != 0)
                {
                    ROS_ERROR_STREAM("FINVECX IS EQ TO FINVECY. terminated.");
                    shutdown();
                }

                if(finvec_x - bufx > 50 || finvec_y - bufy > 50) //DEBUG rerror
                {
                    ROS_ERROR_STREAM("FINVEC IS EXCEEDING LIMITS AND AVOIDS FILTRATION --> " << std::endl << "finvec_x ---> " << finvec_x << std::endl << "finvec_y ---> " << finvec_y);
                    shutdown();
                }
            }
        }
        if(0)
        {
            ROS_WARN_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl << "FUNCTION NAME: result vector in variables" << std::endl << "VARIABLES: list of vars:" << std::endl << "finvec_x -->" << finvec_x << std::endl << "finvec_y -->" << finvec_y << std::endl << "vector length -->" << sqrt(pow(finvec_x , 2) + pow(finvec_y , 2)) <<  std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl);
        }   
        // CODEINFO angle calculation to pass to rviz markers ---> calculate current(i) vector's angle 
        angles.yaw = (atan(finvec_y/finvec_x)/* - (M_PI) * (finvec_x > 0)*/); //BUG no vector inversion
        angles.yaw = (angles.yaw + 2 * M_PI) * (angles.yaw < - M_PI) + (angles.yaw) * (!(angles.yaw < - M_PI));

        tf2::Quaternion q;

        //IDEA make a bullshit filter for potential fields and create max passed speed constant
        q.setRPY(angles.roll, angles.pitch, angles.yaw - (M_PI) * (finvec_x > 0));
        q = q.normalize(); //SOLVED something is going with angle. cant say what, maybe its not a bug ---> in velocity calculation there was an if statement that tried to pass only > 0.001 coordiantes. It totally deleted all negative values from coordinates

        // DEBUG INFO rinfo
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
        final_vel_arrow.set_frame("laser"); //FRAME
        final_vel_arrow.set_namespace("speeds_namespace");
        final_vel_arrow.set_id(1);
        final_vel_arrow.set_type(0);
        final_vel_arrow.set_pose(0, 0, 0, q);
        final_vel_arrow.set_scale(-sqrt(pow(finvec_x, 2) + pow(finvec_y, 2)), 0.05, 0.05);
        final_vel_arrow.set_color(1.0, 0.0, 1.0, 0.0);

        if(publish_rviz_vizualization == 1)
        {
            final_vel_arrow.publish_marker(); // PUB //RVIZ publish
        }
        //*-*-*-*-*-*-*-*--*-*-*-*-*--*--*-*-*-*-*-*-*--*--*-*-*-*-*-*-*-*-*-*-*--*-*-*-*
    }
};


class SpeedRegulator2D
{
    private:
    int regulator_mode; //1 - wheeled bot, 2 - quadrocopter
    double x_regulated;
    double y_regulated;
    float maxRange; //max range to calculate
    MarkerHandler visibility_circle;
    std::string visibility_circle_topic;
    int publish_rviz_vizualization;
    ros::Publisher speed_pub;
    ros::NodeHandle vlcts;


    public:
    SpeedRegulator2D()
    {
        regulator_mode = 1;
        x_regulated = 0;
        y_regulated = 0;
        visibility_circle_topic = "/visibility_circle";
        visibility_circle.advertise_to_topic(visibility_circle_topic);
        publish_rviz_vizualization = 1;
        speed_pub = vlcts.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
        maxRange = 1;
    }

    void set_regulate_one_vector(double &x, double &y)
    {
        x_regulated += x;
        y_regulated += y;
        x = 0;
        y = 0;
    }

    void set_regulate_one_vector_OVERRIDE(double &x, double &y)
    {
        x_regulated = x;
        y_regulated = y;
        x = 0;
        y = 0;
    }

    void set_sum_two_vectors_to_regulate(double x1, double &y1, double x2, double y2)
    {
        x_regulated += x1 + x2;
        y_regulated += y1 + y2;
    }

    void regulate()
    {
        EulerAngles angles;
        angles.yaw = (atan(y_regulated/x_regulated)/* - (M_PI) * (finvec_x > 0)*/); //BUG no vector inversion
        angles.yaw = (angles.yaw + 2 * M_PI) * (angles.yaw < - M_PI) + (angles.yaw) * (!(angles.yaw < - M_PI));

        //CODEINFO Twist init and pub
        geometry_msgs::Twist twist;
        if(sqrt(pow(x_regulated, 2) + pow(y_regulated, 2)) < 0.01)
        {
            twist.linear.x = 0;
            twist.linear.y = 0;
            twist.angular.z = 0;
            ROS_ERROR_STREAM(std::endl << "Speed vector is too small" << std::endl);
        }
        else
        {
            if(abs(angles.yaw) - 0.2 < 0)
            {
                twist.angular.z = 0;
                twist.linear.x = x_regulated;
                twist.linear.y = y_regulated;
            }


            else if (angles.yaw > 0 && angles.yaw < M_PI && regulator_mode == 1)
            {
                ROS_ERROR_STREAM(std::endl << "Angle is correlating badly" << std::endl << "tan --> " << y_regulated/x_regulated << std::endl << "yaw --> " << angles.yaw * 180 / M_PI << " degrees." << std::endl);
                twist.angular.z = 0.4;
            }

            else if (angles.yaw < 0 && angles.yaw >= -M_PI && regulator_mode == 1)
            {
                ROS_ERROR_STREAM(std::endl << "Angle is correlating badly" << std::endl << "tan --> " << y_regulated/x_regulated << std::endl << "yaw --> " << angles.yaw * 180 / M_PI << " degrees." << std::endl);
                twist.angular.z = -0.4;
            }
            else
            {
                //DEBUG INFO rerror
                //****************************************************************************************************
                if(1)
                {
                    ROS_ERROR_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl 
                    << "FUNCTION NAME: some shit is happening" << std::endl 
                    << "VARIABLES: "
                    << "angles.yaw -->" << angles.yaw << std::endl 
                    << "regulator_mode -->" << regulator_mode << std::endl 
                    << "M_PI -->" << M_PI << std::endl 
                    << "_________________________________" << std::endl << "_________________________________" << std::endl);
                }
                //****************************************************************************************************
            }
        }
        speed_pub.publish(twist); //PUB twist
        x_regulated = 0;
        y_regulated = 0;

        //* - * - * - * - * - * - * - * - * - * - * - */ - */ - * - * - * - * - *
        //* - * - * - * - * - * -     RVIZ      - * - */ - */ - * - * - * - * - *
        //* - * - * - * - * - * - * - * - * - * - * - */ - */ - * - * - * - * - *
        tf2::Quaternion q;
        q.setX(0);
        q.setY(0);
        q.setZ(0);
        q.setW(1);

        visibility_circle.set_frame("laser"); //FRAME
        visibility_circle.set_namespace("speeds_namespace");
        visibility_circle.set_id(2);
        visibility_circle.set_type(3);
        visibility_circle.set_pose(0, 0, -0.2, q);
        visibility_circle.set_scale(2 * maxRange, 2 * maxRange, 0.05);
        visibility_circle.set_color(0.3, 0.0, 0.0, 1.0);

        if(publish_rviz_vizualization == 1)
        {
            visibility_circle.publish_marker(); // PUB //RVIZ publish
        }
        //* - * - * - * - * - * - * - * - * - * - * - */ - */ - * - * - * - * - *
        //* - * - * - * - * - * - * - * - * - * - * - */ - */ - * - * - * - * - *
    }
};

PotentialFieldRepulsive *repulse_pointer;
SpeedRegulator2D *regulator_pointer;

// PRESETUP 
bool publish_rviz_vizualization = 1;
int okcheck = 1;

// SUB catchPC creation
ros::Subscriber catchPC_sub;
ros::Publisher Catched_Cloud_and_PF;// PUB declaration

void got_scanCallback(const sensor_msgs::PointCloud::ConstPtr& catchedCloud)
{
    repulse_pointer->calculate_repulse_vector(catchedCloud);
    regulator_pointer->set_regulate_one_vector(repulse_pointer->finvec_x, repulse_pointer->finvec_y);
    regulator_pointer->regulate();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "potential_fields");

    PotentialFieldRepulsive repulse;
    SpeedRegulator2D regulator;

    repulse_pointer = &repulse;
    regulator_pointer = &regulator;

    // NODEHANDLE
    ros::NodeHandle cs;

    //PUB adverts
    catchPC_sub = cs.subscribe("/transPC", 10, got_scanCallback); // SUB catchPC sub

    ros::spin();

    return 0;
}
