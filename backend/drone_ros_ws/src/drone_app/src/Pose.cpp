#include "common/Pose.hpp"

std::ostream& operator<<(std::ostream& out, const Pose& p)
{
    return out << p.posX << ", " << p.posY << ", " << p.posZ << ", "
               << p.rotX << ", " << p.rotY << ", " << p.rotZ;
}

Pose& Pose::operator+=(const Pose& other)
{
    this->posX += other.posX;
    this->posY += other.posY;
    this->posZ += other.posZ;

    this->rotX += other.rotX;
    this->rotY += other.rotY;
    this->rotZ += other.rotZ;

    return *this;
}

Pose Pose::operator+(const Pose& other) const
{
    Pose res = *this;

    return res += other;
}
