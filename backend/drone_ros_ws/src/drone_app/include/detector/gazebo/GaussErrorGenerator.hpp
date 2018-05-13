#ifndef GAUSS_ERROR_GENERATOR_HPP_
#define GAUSS_ERROR_GENERATOR_HPP_

#include "IErrorGenerator.hpp"

#include <random>

/**
 *  Generates an error with the Gauss distribution,
 *  with a given pose as the mean value.
 */
class GaussErrorGenerator : public IErrorGenerator
{
public:
    /**
     *  Constructor.
     *  
     *  \param [in] mean Mean of the normal distribution
     *  \param [in] std_dev Standard deviation of the normal distribution
     */
    GaussErrorGenerator(double mean, double std_dev);

    /**
     *  Generates a pose with the random values.
     *  
     *  \return A Pose object with the random values
     */
    virtual Pose generatePose();
    
    /**
     *  Generates a single random value.
     *  
     *  \return A random number
     */
    virtual double generateValue();

    /**
     *  Destructor.
     */
    virtual ~GaussErrorGenerator() {}

private:
    std::mt19937 m_generator;

    double m_mean;

    double m_std_dev;
};

#endif /* GAUSS_ERROR_GENERATOR_HPP_ */
