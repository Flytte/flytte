#ifndef IERROR_GENERATOR_HPP_
#define IERROR_GENERATOR_HPP_

#include "common/Pose.hpp"

/**
 *  Interface for an error generator.
 */
class IErrorGenerator
{
public:
    /**
     *  Generates a pose with the random values.
     *  
     *  \return A Pose object with the random values
     */
    virtual Pose generatePose() = 0;

    /**
     *  Generates a single random value.
     *  
     *  \return A random number
     */
    virtual double generateValue() = 0;

    /**
     *  \brief Destructor.
     */
    virtual ~IErrorGenerator() {}
};

#endif /* IERROR_GENERATOR_HPP_ */
