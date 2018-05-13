#ifndef BBOX_HPP_
#define BBOX_HPP_

#include <iostream>

/**
 *  Describes a bounding box, that is a position in the coordinate system
 *  of the camera.
 */
struct BBox
{
    BBox(double posX = 0, double posY = 0, double w = 0, double h = 0,
        double rotX = 0, double rotY = 0, double rotZ = 0) :
        posX(posX),
        posY(posY),
        w(w),
        h(h),
        rotX(rotX),
        rotY(rotY),
        rotZ(rotZ) {}

    friend std::ostream& operator<<(std::ostream& out, const BBox& p);

    double posX;
    double posY;
    double w;
    double h;
    double rotX;
    double rotY;
    double rotZ;
};

#endif /* BBOX_HPP_ */
