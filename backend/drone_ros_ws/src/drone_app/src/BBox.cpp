#include "common/BBox.hpp"

std::ostream& operator<<(std::ostream& out, const BBox& p)
{
    return out << p.posX << ", " << p.posY << ", " << p.w << ", " << p.h << ", "
               << p.rotX << ", " << p.rotY << ", " << p.rotZ;
}
