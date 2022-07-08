#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>

int counter = 0, counter_buff = 1;
double pose_x = 0.0, pose_y = 0.0;
ros::Publisher goal_const_pub;
geometry_msgs::PoseStamped goal_pose;


void transform_initialiser_callback(const geometry_msgs::PoseStamped::ConstPtr& catched_const_goal)
{
    while(counter == counter_buff)
    {
        static tf2_ros::TransformBroadcaster br; // Создание объекта транслятора трансформаций
        geometry_msgs::TransformStamped transformStamped; // Создание объекта трансформации

        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "odom"; // задается фрейм-отец
        transformStamped.child_frame_id = "nav_goal_frame"; // задается фрейм-ребенок
        transformStamped.transform.translation.x = catched_const_goal->pose.position.x; // Задаются координаты смещения нового фрейма относительно фрейма-отца
        transformStamped.transform.translation.y = catched_const_goal->pose.position.y;
        transformStamped.transform.translation.z = 0.0;
        tf2::Quaternion q; // Задается поворот(в кватернионе) фрейма-ребенка относительно фрейма-отца
        q.setRPY(0, 0, 0);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();

        //DEBUG rinfo
        //****************************************************************************************************
        if(1)
        {
            ROS_INFO_STREAM(std::endl << "_________________________________" << std::endl << "_________________________________" << std::endl 
            << "FUNCTION NAME: poses" << std::endl 
            << "VARIABLES: "<< std::endl 
            << "pose x -->" << pose_x << std::endl 
            << "pose y -->" << pose_y << std::endl 
            << "_________________________________" << std::endl << "_________________________________" << std::endl);
        }
        //****************************************************************************************************

        br.sendTransform(transformStamped); // Трансформация отправляется
        counter_buff = counter;
    }
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& catched_goal) // Колбэк получает из топика turtle_name+"/pose" сообщение позиции, определенное в библиотеке turtlesim. turtle_name - параметр имени конкретной черепашки, который позволяет вазимодействовать не только с первой созданной черепашкой, но и с последующими
{
    // pose_x = catched_goal->pose.position.x;
    // pose_y = catched_goal->pose.position.y;
    goal_pose.pose.position.x = catched_goal->pose.position.x;
    goal_pose.pose.position.y = catched_goal->pose.position.y;
    goal_pose.pose.position.z = catched_goal->pose.position.z;
    goal_const_pub.publish(goal_pose);
    counter++;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "my_tf2_broadcaster");
    
    ros::NodeHandle node;
    ros::NodeHandle node_parallel;
    
    ros::Subscriber original_goal_sub = node.subscribe("/move_base_simple/goal", 10, poseCallback); 
    goal_const_pub = node_parallel.advertise<geometry_msgs::PoseStamped>("/attraction_goal", 1000);

    ros::Subscriber const_goal_sub = node_parallel.subscribe("/attraction_goal", 1000, transform_initialiser_callback);

    ros::spin();
    return 0;
};