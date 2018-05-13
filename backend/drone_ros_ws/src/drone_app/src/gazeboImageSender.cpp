#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include "gazebo/transport/transport.hh"
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/common/Image.hh>

#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>

image_transport::Publisher pub;

void callback(ConstImageStampedPtr& _msg)
{
    std_msgs::String msg;

    int width  = _msg->image().width();
    int height = _msg->image().height();
    int step   = _msg->image().step();
    std::string data = _msg->image().data();

    sensor_msgs::Image img;

    img.height = height;
    img.width = width;
    img.step = step;
    img.encoding = "rgb8";

    for(unsigned int i = 0; i < data.length(); i++)
    {
        img.data.push_back(data[i]);
    }

    pub.publish(img);
}

int main(int argc, char** argv)
{
    // ROS initialization

    ros::init(argc, argv, "gazeboImageSender");
    ros::NodeHandle n;
    image_transport::ImageTransport it(n);

    // Gazebo initialization

    gazebo::client::setup(argc, argv);
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    // Topics

    pub = it.advertise("camera_img", 1);

    gazebo::transport::SubscriberPtr sub = node->Subscribe("~/camera/camera_link/camera1/image", callback);

    // Main loop

    while(ros::ok())
    {
        ros::spinOnce();
    }

    // Shutdown

    gazebo::client::shutdown();

    return 0;
}
