#include <tf/transform_datatypes.h>

#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include "common/Pose.hpp" 

#include <iostream>

void cb(ConstPosesStampedPtr& _msg)
{
    constexpr int NUM = 2;

    Pose p;

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

    std::cout << p << std::endl;
}

int main(int _argc, char** _argv)
{
    // Load gazebo
    gazebo::client::setup(_argc, _argv);

    // Create our node for communication
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Header
    std::cout << "posX, posY, posZ, rotX, rotY, rotZ" << std::endl;

    // Listen to Gazebo world_stats topic
    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/pose/info", cb);

    // Busy wait loop...replace with your own code as needed.
    while(true)
    {
        gazebo::common::Time::MSleep(10);
    }
    // Make sure to shut everything down.
    gazebo::client::shutdown();

    return 0;
}
