#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <iostream>
#include <fstream>


#include "common/BBox.hpp"
#include "common/Pose.hpp"

#include "controller/Math.hpp"
#include "controller/Drone.hpp"
#include "detector/gazebo/random.hpp"


#include "drone_app_msgs/DroneArray.h"

ros::Publisher pub;

IErrorGenerator* positionErrorGenerator;
IErrorGenerator* orientationErrorGenerator;

Math* math;

void initErrorModel()
{
    double mean;
    double std_dev;

    if(!ros::param::get("/simulation/models/gauss/mean", mean) ||
       !ros::param::get("/simulation/models/gauss/std_dev", std_dev))
    {
        throw(std::runtime_error("Mean or standard deviation for Gauss error model are not set"));
    }

    orientationErrorGenerator = new GaussErrorGenerator(mean, std_dev);

    int rate;
    double min;
    double max;

    if(!ros::param::get("/simulation/models/statistical/rate", rate) ||
       !ros::param::get("/simulation/models/statistical/min", min)   ||
       !ros::param::get("/simulation/models/statistical/max", max))
    {
        throw(std::runtime_error("Min or max for statistical error model are not set"));
    }

    positionErrorGenerator = new StatisticalErrorGenerator(rate, min, max);
}

void callback(ConstPosesStampedPtr& _msg)
{
    drone_app_msgs::Drone drone;
    drone_app_msgs::DroneArray msg;

    Pose p;

    constexpr int NUM = 2;

    p.posX = _msg->pose(NUM).position().x();
    p.posY = _msg->pose(NUM).position().y();
    p.posZ = _msg->pose(NUM).position().z();
    p.rotX = _msg->pose(NUM).orientation().x();
    p.rotY = _msg->pose(NUM).orientation().y();

    tf::Quaternion q(
        _msg->pose(NUM).orientation().x(),
        _msg->pose(NUM).orientation().y(),
        _msg->pose(NUM).orientation().z(),
        _msg->pose(NUM).orientation().w()
    );

    p.rotZ = tf::getYaw(q);

    // Add an artificial error
    //p += positionErrorGenerator->generatePose();
    //p += orientationErrorGenerator->generatePose();

    // Transform it to a BBox
    Drone dr("parrot_bebop2");
    BBox b = math->pose2BBox(dr, math->rotate2ClientSimulation(p));

    // Add to an array and send
    drone.id              = -1;
    drone.name            = "parrot_bebop2";
    drone.box.w           = b.w;
    drone.box.h           = b.h;
    drone.box.t.linear.x  = b.posX;
    drone.box.t.linear.y  = b.posY;
    drone.box.t.linear.z  = 0;
    drone.box.t.angular.x = b.rotX;
    drone.box.t.angular.y = b.rotY;
    drone.box.t.angular.z = b.rotZ;

    msg.drones.push_back(drone);

    pub.publish(msg);
}

int main(int argc, char** argv)
{
    // ROS initialization
    ros::init(argc, argv, "gazeboDetector");
    ros::NodeHandle n;

    // Gazebo initialization
    gazebo::client::setup(argc, argv);
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Error model parameters
    initErrorModel();

    // Getting access to Math Class
    math = new Math();

    // Topics
    pub = n.advertise<drone_app_msgs::DroneArray>("simulation_drones", 1000);
    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose/info", callback);

    // Main loop
    while(ros::ok())
    {
        ros::spinOnce();
    }

    // Shutdown
    gazebo::client::shutdown();

    delete positionErrorGenerator;
    delete orientationErrorGenerator;
    delete math;

    return 0;
}
