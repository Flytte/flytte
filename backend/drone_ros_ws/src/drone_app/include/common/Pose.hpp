#ifndef POSE_HPP_
#define POSE_HPP_

#include <iostream>

/**
 *  Describes a pose, that is a 6D vector with 3 position coordinates
 *  as well as 3 orientation values.
 */
struct Pose
{
    Pose(double posX = 0, double posY = 0, double posZ = 0,
        double rotX = 0, double rotY = 0, double rotZ = 0) :
        posX(posX),
        posY(posY),
        posZ(posZ),
        rotX(rotX),
        rotY(rotY),
        rotZ(rotZ) {}

    Pose& operator+=(const Pose& other);

    Pose operator+(const Pose& other) const;

    friend std::ostream& operator<<(std::ostream& out, const Pose& p);

    double posX;
    double posY;
    double posZ;
    double rotX;
    double rotY;
    double rotZ;
};

#endif /* POSE_HPP_ */
