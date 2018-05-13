#include <ros/ros.h>
#include <vector>

#include "controller/DronePool.hpp"
#include "controller/Math.hpp"
#include "controller/RegulatorUtil.hpp"

#include "drone_app_msgs/DroneArray.h"
#include "geometry_msgs/Twist.h"

#include <iostream>

// --- Needed for latency test ---
#include <chrono>
#include <thread>

int useLatency ;
int latencyTime;      // ms
// ------------------------------

DronePool pool;

ros::Publisher cmd_vel;
ros::Publisher client;

// This part is meant to be used when having multiple drones
// std::unordered_map<std::string, ros::Publisher> cmd_topics;

// Heuristic. Will be changed later
int findID(std::string name)
{
    for(std::pair<unsigned int, Drone&> pair : pool.all())
    {
        if(pair.second.name() == name)
        {
            return pair.first;
        }
    }

    return -1;
}


void update(const drone_app_msgs::DroneArray::ConstPtr& msg)
{
    // Update poses of all the drones
    for(unsigned int i = 0; i < msg->drones.size(); i++)
    {
        const drone_app_msgs::Drone& detectedDrone = msg->drones[i];

        BBox box(
            detectedDrone.box.t.linear.x,
            detectedDrone.box.t.linear.y,
            detectedDrone.box.w,
            detectedDrone.box.h,
            detectedDrone.box.t.angular.x,
            detectedDrone.box.t.angular.y,
            detectedDrone.box.t.angular.z);

        int id = findID(detectedDrone.name);
        if(id < 0)
        {
            id = pool.createDrone(
                detectedDrone.name,
                box);
        }
        else
        {
            Drone& drone = pool.findDrone(id);
            drone.setCurrent(box);
        }
    }

    // Send all the drone in the pool to the client
    drone_app_msgs::DroneArray toClient;

    for(auto pair : pool.all())
    {
        drone_app_msgs::Drone droneMsg;

        unsigned int id = pair.first;
        Drone& drone = pair.second;
        BBox box = drone.current().first;

        droneMsg.id              = id;
        droneMsg.name            = drone.name();
        droneMsg.box.w           = box.w;
        droneMsg.box.h           = box.h;
        droneMsg.box.t.linear.x  = box.posX;
        droneMsg.box.t.linear.y  = box.posY;
        droneMsg.box.t.linear.z  = 0;
        droneMsg.box.t.angular.x = box.rotX;
        droneMsg.box.t.angular.y = box.rotY;
        droneMsg.box.t.angular.z = box.rotZ;

        toClient.drones.push_back(droneMsg);
    }

    // Latency (Server -> Client)
    if(useLatency)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(latencyTime));
    }

    client.publish(toClient);
}

void handleCommand(const drone_app_msgs::BBox::ConstPtr& msg)
{
    // Latency (Client -> Server)
    if(useLatency)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(latencyTime));
    }

    if(!pool.hasDrone(msg->id))
    {
        ROS_ERROR("Drone not found. ID = %d", msg->id);
        return;
    }

    Drone& drone = pool.findDrone(msg->id);

    BBox box(
        msg->t.linear.x,
        msg->t.linear.y,
        msg->w,
        msg->h,
        msg->t.angular.x,
        msg->t.angular.y,
        msg->t.angular.z);

    drone.setTarget(box);
}

void loop(Math& math /*, ros::NodeHandle& n*/)
{
    for(std::pair<unsigned int, Drone&> pair : pool.all())
    {
        unsigned int id = pair.first;
        Drone& drone = pair.second;

        if(!drone.last().second || !drone.target().second)
        {
            continue;
        }

        // Regulator

        RegulatorResult res;

        try
        {
            res = RegulatorUtil::checkState(math, drone);
        }
        catch(std::runtime_error e)
        {
            ROS_ERROR("Regulator error: Drone ID = %u, name = %s. What: %s",
                        id, drone.name().c_str(), e.what());

            res = RegulatorResult::ANOMALY;
        }

        // Creating the command

        geometry_msgs::Twist cmd;

        if(res == RegulatorResult::ARRIVED)
        {
            // Stop the drone, clear the target

            cmd.linear.x  =
            cmd.linear.y  =
            cmd.linear.z  =
            cmd.angular.x =
            cmd.angular.y =
            cmd.angular.z = 0;
            std::cout<<"ARR"<<std::endl;
            drone.clearTarget();
        }
        else if(res == RegulatorResult::ANOMALY ||
                res == RegulatorResult::WRONG_DIRECTION ||
                res == RegulatorResult::PASSED_TARGET)
        {
            // Stop the drone but don't clear the target

            cmd.linear.x  =
            cmd.linear.y  =
            cmd.linear.z  =
            cmd.angular.x =
            cmd.angular.y =
            cmd.angular.z = 0;
            std::cerr<<"ANO"<<std::endl;
        }
        else
        {
            // Calculate the command vector

            Pose cmd_pose = math.transform(drone);

            cmd.linear.x  = cmd_pose.posX;
            cmd.linear.y  = cmd_pose.posY;
            cmd.linear.z  = cmd_pose.posZ;
            cmd.angular.x = 0;
            cmd.angular.y = 0;
            cmd.angular.z = cmd_pose.rotZ;
            std::cout<<"OK"<<std::endl;
        }

        // Sending the command

        cmd_vel.publish(cmd);

// This part is meant to be used when having multiple drones
/*
        std::string topic_name;
        if(!ros::param::get("/drones/" + drone.name() + "/topics/commands", topic_name))
        {
            ROS_ERROR("Command topic not found: Drone ID = %u, name = %s",
            id, drone.name().c_str());

            continue;
        }

        auto it = cmd_topics.find(topic_name);
        if(it == cmd_topics.end())
        {
            ros::Publisher topic = n.advertise<geometry_msgs::Twist>(topic_name, 1000);
            cmd_topics.insert(std::pair<std::string, ros::Publisher>(topic_name, topic));
            topic.publish(cmd);
        }
        else
        {
            it->second.publish(cmd);
        }
*/
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "controller");
    ros::NodeHandle n;

    ros::Subscriber subFixedDrones = n.subscribe("fixed_drones", 1000, update);
    ros::Subscriber subCommands = n.subscribe("commands", 1000, handleCommand);

    cmd_vel = n.advertise<geometry_msgs::Twist>("bebop/cmd_vel", 1000);
    client  = n.advertise<drone_app_msgs::DroneArray>("client_drones", 1000);

    ros::WallRate loop_rate(10);

    Math math;

    // Latency
    if(!ros::param::get("/latency/time", latencyTime)
    || !ros::param::get("/latency/use", useLatency))
    {
        ROS_ERROR("Latency parameters could not be read");
        return -1;
    }

    while(ros::ok())
    {
        loop(math /*, n*/);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
