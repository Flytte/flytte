#ifndef REGULATOR_UTIL_HPP_
#define REGULATOR_UTIL_HPP_

#include "controller/DronePool.hpp"
#include "controller/Math.hpp"

/**
 *  Utility class for detection of the drone's anomaly positions.
 */
class RegulatorUtil
{
public:

    /**
     *  Describes possible output of the regulator.
     */
    enum class RegulatorResult
    {
        OK,
        ARRIVED,
        WRONG_DIRECTION,
        PASSED_TARGET,
        ANOMALY
    };

    /**
     *  Checks the position of the drone relatively to its target position.
     *  
     *  \param [in] math Math object needed to perform mathematical calculations
     *  \param [in] drone Drone which position has to be evaluated
     *  
     *  \return OK if the drone is on the way to its target position,
     *          ARRIVED if the drone's current position is its target position,
     *          WRONG_DIRECTION if the drone is flying to an incorrect direction,
     *          ANOMALY if the calculations can't be performed (e.g. width or height of the bounding box is zero)
     */
    static RegulatorResult checkState(Math& math, Drone& drone);

private:
    static constexpr double DELTA   = 0.3;
    static constexpr double EPSILON = 1.0;
    static constexpr double SHAKE = 0.05;
};

typedef RegulatorUtil::RegulatorResult RegulatorResult;

#endif /* REGULATOR_UTIL_HPP_ */
