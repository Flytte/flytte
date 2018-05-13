#ifndef STATISTICAL_ERROR_GENERATOR_HPP_
#define STATISTICAL_ERROR_GENERATOR_HPP_

#include "IErrorGenerator.hpp"

#include <random>

/**
 *  Generates an error with the uniform distribution,
 *  with an average rate.
 */
class StatisticalErrorGenerator : public IErrorGenerator
{
public:
    /**
     *  Constructor.
     *  
     *  \param [in] rate Probability of a random number to be generated is inverse proportional to rate
     *  \param [in] min Minimal value of the generated random number
     *  \param [in] max Maximal value of the generated random number
     */
    StatisticalErrorGenerator(int rate, double min, double max);
    
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
    virtual ~StatisticalErrorGenerator() {}

private:
    std::mt19937 m_generator;

    int m_rate;

    double m_min;

    double m_max;
};

#endif /* STATISTICAL_ERROR_GENERATOR_HPP_ */
