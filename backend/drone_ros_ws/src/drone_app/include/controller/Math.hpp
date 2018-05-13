#ifndef MATH_HPP_
#define MATH_HPP_

#include <ros/ros.h>

#include <math.h>
#include <string.h>
#include <unordered_map>

#include "controller/Drone.hpp"

/**
* Holds all the camera parameters
*/
struct CameraParams
{
    // --- Camera position (in simulation)
    double x;
    double y;
    double z;

    // --- Focal length (in pixel units)
    double fx;
    double fy;

    // --- Optical center (in pixels)
    double cx;
    double cy;

    // --- Image size (in pixels)
    double w;
    double h;
};

/**
* Holds all the drone parameters
*/
struct DroneParams
{
    // --- Drone's size
    double w;
    double h;
};

/**
* Holds all the drone velocity parameters
*/
struct DroneVelocityParams
{
    // --- Max velocity quotient
    double max_h;
    double max_v;
};

/**
* This class offers a set of useful functions for transformation between
* coordinate systems, distances
*/
class Math
{

public:
    /**
    * Default constructor
    */
    Math();

    /**
    * Transforms target position in a velocity vector
    * @param dr     holds a drone reference
    * @return the velocity vector
    */
    Pose transform(Drone& dr);

    /**
    * Computes a distance between two BBoxes
    * @param dr     holds a drone reference
    * @param from   holds a BBox Kind from which the distance is calculated
    * @param to     holds a BBox Kind to which the distance is calculated
    * @return the distance
    */
    double distance(    Drone& dr,
                        Drone::BBoxKind from,
                        Drone::BBoxKind to);

    /**
    * Computer intersection over union of 2 BBoxes, which can be used to
    * compute how much they overlap
    * @param from   holds a BBox from
    * @param to     holds a BBox to
    * @return the iou value
    */
    double iou(BBox from, BBox to);

    /**
    * Computes a vector, that points from the camera to the drone's position
    * @param dr     holds a drone reference
    * @param to     holds a BBox Kind to which the vector should be calcualted
    * @return the vector to position
    */
    Pose distanceVector(  Drone& dr,
                          Drone::BBoxKind to);

    /**
    * Computes a vector, that points from the current_pose to target_pose
    * @param from    starting pose
    * @param to      target pose
    * @return the vector to target_pose
    */
    Pose directionVector( Pose from,
                          Pose to);

    /**
    * Rotates the vector to the drone coordinates, given it's current position
    * @param current_pose     drone's current position
    * @param vector           vector to be rotated
    * @return the rotated vector
    */
    Pose rotate2Drone(  Pose current_pose,
                        Pose vector);

    /**
    * Rotates a drone's pose from the simulation to the client
    * @param current_pose     drone's current position
    * @return the rotated pose
    */
    Pose rotate2ClientSimulation(Pose current_pose);

    /**
    * Pushes the vector values into [-1,1] interval
    * @param vector     vector
    * @return the normalized vector
    */
    Pose normalize( Pose vector);

    /**
    * Convert's a Pose into BBox
    * @param dr     holds a drone reference
    * @param pose   pose
    * @return the bounding box
    */
    BBox pose2BBox( Drone& dr,
                    Pose pose);

private:
    // Gets drone's paramters from the parameter server
    DroneParams getDroneParams(std::string droneName);

    CameraParams m_camera;
    DroneVelocityParams m_drone_vel;

    std::unordered_map<std::string, DroneParams> m_cache;
};

#endif /* MATH_HPP_ */
