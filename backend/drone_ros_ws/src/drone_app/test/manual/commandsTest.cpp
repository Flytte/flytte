#include<ros/ros.h>
#include<std_msgs/Empty.h>
#include<geometry_msgs/Twist.h>

#include <iostream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "commandsTest");
    ros::NodeHandle n;

    ros::Publisher takeoff = n.advertise<std_msgs::Empty>("/bebop/takeoff", 1000, true);
    ros::Publisher cmd_vel = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel", 1000);
    ros::Publisher land = n.advertise<std_msgs::Empty>("/bebop/land", 1000, true);

    ros::Rate loop_rate(10);

    std_msgs::Empty empty;
    geometry_msgs::Twist cmd;

    cmd.linear.x  = 1;
    cmd.linear.y  = 0;
    cmd.linear.z  = 0;
    cmd.angular.x = 0;
    cmd.angular.y = 0;
    cmd.angular.z = 0;

    std::cout << "Taking off..." << std::endl;

    while(takeoff.getNumSubscribers() <= 0)
    {
        std::cout << "Waiting for takeoff topic..." << std::endl;
    }

    while(land.getNumSubscribers() <= 0)
    {
        std::cout << "Waiting for land topic..." << std::endl;
    }

    takeoff.publish(empty);

    std::cout << "In the air!" << std::endl;

    while(ros::ok())
    {
        std::cout << "Flying..." << std::endl;

        cmd_vel.publish(cmd);

        std::cout << "Done!" << std::endl;
        
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << "Landing..." << std::endl;

    land.publish(empty);

    std::cout << "Landed!" << std::endl;

    return 0;
}
