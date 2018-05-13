#include "controller/Math.hpp"

Math::Math()
{
    if(
        !ros::param::get("/camera/x", m_camera.x) ||
        !ros::param::get("/camera/y", m_camera.y) ||
        !ros::param::get("/camera/z", m_camera.z))
    {
        throw(std::runtime_error("Camera position not found"));
    }

    if(
        !ros::param::get("/camera/fx", m_camera.fx) ||
        !ros::param::get("/camera/fy", m_camera.fy) ||
        !ros::param::get("/camera/cx", m_camera.cx) ||
        !ros::param::get("/camera/cy", m_camera.cy))
    {
        throw(std::runtime_error("Camera matrix is not found"));
    }

    if(
        !ros::param::get("/camera/w", m_camera.w) ||
        !ros::param::get("/camera/h", m_camera.h))
    {
        throw(std::runtime_error("Image size is not found"));
    }

    if(
        !ros::param::get("/drone_velocity/max_h", m_drone_vel.max_h) ||
        !ros::param::get("/drone_velocity/max_v", m_drone_vel.max_v))
    {
        throw(std::runtime_error("Drone velocities are not found"));
    }
}

DroneParams Math::getDroneParams(std::string droneName)
{
    DroneParams res;

    auto it = m_cache.find(droneName);
    if(it == m_cache.end())
    {
        // Cache miss
        double width;
        double height;

        std::string param_name = "/drones/" + droneName + "/size";
        if(
            !ros::param::get(param_name + "/width", width) ||
            !ros::param::get(param_name + "/height", height))
        {
            throw(std::runtime_error("Drone's parameters not found"));
        }

        // Add to cache
        res = {width, height};
        std::pair<std::string, DroneParams> pair(droneName, res);

        m_cache.insert(pair);
    }
    else
    {
        // Cache hit

        res = it->second;
    }

    return res;
}

Pose Math::distanceVector(  Drone& dr,
                            Drone::BBoxKind to)
{
    DroneParams droneSize = getDroneParams(dr.name());

    std::pair<BBox, bool> to_pair = dr.getBBox(to);

    if(std::get<1>(to_pair))
    {
      BBox bb = std::get<0>(to_pair);

      if( bb.w == 0 || bb.h == 0 )
      {
          throw(std::runtime_error("BBox of size zero"));
      }

      // --- Convert percentage to pixels
      bb.posX = bb.posX * m_camera.w / 100;
      bb.posY = bb.posY * m_camera.h / 100;
      bb.w = bb.w * m_camera.w / 100;
      bb.h = bb.h * m_camera.h / 100;

      // --- Move to drone center
      bb.posX += bb.w / 2;
      bb.posY += bb.h / 2;

      // --- Convert BBox to Pose
      Pose result;
      result.posX = (droneSize.w * (bb.posX - m_camera.cx)) / bb.w;
      result.posY = (droneSize.h * (bb.posY - m_camera.cy)) / bb.h;
      result.posZ = droneSize.w * m_camera.fx / bb.w;
      result.rotZ = bb.rotZ;

      return result;
    }
    else
    {
        throw(std::runtime_error("BBox is not defined"));
    }
}

Pose Math::rotate2Drone(  Pose current_pose,
                          Pose vector)
{
    // --- Convert degrees to radians
    double angle = current_pose.rotZ * M_PI / 180.0;

    Pose result;

    // --- Rotate the vector
    result.posX = (-1) * sin(angle) * vector.posX + cos(angle) * vector.posZ;
    result.posY = (-1) * cos(angle) * vector.posX - sin(angle) * vector.posZ;
    result.posZ = (-1) * vector.posY;

    // --- Save rotation as it is
    result.rotZ = vector.rotZ;

    return result;
}

Pose Math::rotate2ClientSimulation(Pose current_pose)
{
    Pose result;

    // Simulation coordinates == Stationary drone coordinates
    result.posX = (-1) * (current_pose.posY - m_camera.y);
    result.posY = (-1) * (current_pose.posZ - m_camera.z);
    result.posZ = (current_pose.posX - m_camera.x);

    // Save rotation in degrees
    result.rotZ = current_pose.rotZ * 180.0 / M_PI;

    return result;
}

BBox Math::pose2BBox( Drone& dr,
                      Pose inClientCoord)
{
    //TODO: Account for distortion

    DroneParams droneSize = getDroneParams(dr.name());

    BBox bb;

    // --- Convert Pose to BBox (pixels)
    bb.posX = (m_camera.fx * (inClientCoord.posX / inClientCoord.posZ) + m_camera.cx);
    bb.posY = (m_camera.fy * (inClientCoord.posY / inClientCoord.posZ) + m_camera.cy);
    bb.w = m_camera.fx * (droneSize.w / inClientCoord.posZ);
    bb.h = m_camera.fy * (droneSize.h / inClientCoord.posZ);

    // --- Convert Pose to BBox (pixels)
    bb.posX = (bb.posX - bb.w/2) * 100 / m_camera.w;
    bb.posY = (bb.posY - bb.h/2) * 100 / m_camera.h;
    bb.w = bb.w * 100 / m_camera.w;
    bb.h = bb.h * 100 / m_camera.h;

    // --- Save rotation as it is
    bb.rotZ = inClientCoord.rotZ;

    return bb;
}

Pose Math::directionVector( Pose current_pose,
                            Pose target_pose)
{
    Pose result;

    result.posX = target_pose.posX - current_pose.posX;
    result.posY = target_pose.posY - current_pose.posY;
    result.posZ = target_pose.posZ - current_pose.posZ;
    result.rotZ = target_pose.rotZ - current_pose.rotZ;

    return result;
}

double Math::distance(  Drone& dr,
                        Drone::BBoxKind from,
                        Drone::BBoxKind to)
{
    // --- Find both distance vectors
    Pose fromDist = distanceVector(dr, from);
    Pose toDist = distanceVector(dr, to);

    // --- Find a direction vector between the too
    Pose vector = directionVector(fromDist, toDist);

    // --- Find the length of the direction vector
    return sqrt(pow(vector.posX, 2) + pow(vector.posY, 2) + pow(vector.posZ, 2));
}

double Math::iou( BBox from,
                  BBox to)
{
    double overlap_area;

    // --- Find a overlap bounding box
    double lX = std::max(from.posX, to.posX);
    double rX = std::min(from.posX + from.w, to.posX + to.w);
    double tY = std::max(from.posY, to.posY);
    double bY = std::min(from.posY + from.h, to.posY + to.h);

    // --- Find its area
    if (lX < rX && tY < bY)
    {
      overlap_area = (rX - lX) * (bY - tY);
    }
    else
    {
      // In case of no overlap
      return 0;
    }

    return overlap_area / (from.w * from.h + to.w * to.h - overlap_area);
}

Pose Math::normalize(Pose vector)
{
    // --- Find max absolute value
    double abs_vector[3] = {std::abs(vector.posX),std::abs(vector.posY),std::abs(vector.posZ)};
    double max = *std::max_element(abs_vector, abs_vector+3);

    Pose result;

    // TODO: parameter server
    result.posX = vector.posX / (m_drone_vel.max_h * max);
    result.posY = vector.posY / (m_drone_vel.max_h * max);
    result.posZ = vector.posZ / (m_drone_vel.max_v * max);
    result.rotZ = vector.rotZ / 360.0;

    return result;
}

Pose Math::transform(Drone& dr)
{
    // --- Find distance vectors
    Pose current_pose = distanceVector(dr, Drone::BBoxKind::CURRENT);
    Pose target_pose = distanceVector(dr, Drone::BBoxKind::TARGET);

    // --- Find a direction vector between them
    Pose direction_vector = directionVector(current_pose, target_pose);

    // --- Rotate this vector to the drone coordinates
    Pose new_coord = rotate2Drone(current_pose, direction_vector);

    // --- Normalize it
    return normalize(new_coord);

}
