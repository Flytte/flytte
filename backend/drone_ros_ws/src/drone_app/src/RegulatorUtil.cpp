#include "controller/RegulatorUtil.hpp"

#include <math.h>

#include <iostream>

using std::sqrt;
using std::pow;

RegulatorResult RegulatorUtil::checkState(Math& math, Drone& drone)
{
    if(!drone.last().second || !drone.target().second)
    {
        // If there are no target or last position we can't say anything

        return RegulatorResult::ANOMALY;
    }

    if( drone.last().first.w == 0 || drone.target().first.w == 0 ||
        drone.last().first.h == 0 || drone.target().first.h == 0)
    {
        // If sizes of BBoxes are zero math doesn't work

        return RegulatorResult::ANOMALY;
    }

    double dLast2Target;
    double dCurrent2Target;
    double dLast2Current;

    dCurrent2Target = math.distance(
                                    drone,
                                    Drone::BBoxKind::CURRENT,
                                    Drone::BBoxKind::TARGET);

    double iou = math.iou(drone.current().first, drone.target().first);

    if(iou > DELTA)
    {
        return RegulatorResult::ARRIVED;
    }
/*
    if(dCurrent2Target < DELTA)
    {
        // If Current position is close enough to target we should stop

        return RegulatorResult::ARRIVED;
    }
*/
    dLast2Target = math.distance(
                                drone,
                                Drone::BBoxKind::LAST,
                                Drone::BBoxKind::TARGET);

    dLast2Current = math.distance(
                                drone,
                                Drone::BBoxKind::LAST,
                                Drone::BBoxKind::CURRENT);

    // if(dLast2Target < dLast2Current)
    // {
    //     // We have passed the target
    //
    //     std::cout << "[D] PASSED" << std::endl;
    //     return RegulatorResult::PASSED_TARGET;
    // }

    if(dLast2Target - dCurrent2Target > SHAKE)
    {
        // We are flying in the opposite direction

        return RegulatorResult::WRONG_DIRECTION;
    }

    double t1 = pow(dCurrent2Target, 2) + pow(dLast2Target, 2) - pow(dLast2Current, 2);
    double t2 = 2 * dLast2Target;

    double normal = sqrt(pow(dCurrent2Target, 2) - pow(t1 / t2, 2));

    if(normal / dLast2Current > EPSILON)
    {
        // We are flying in a wrong (but not opposite) direction

        return RegulatorResult::WRONG_DIRECTION;
    }

    return RegulatorResult::OK;
}
