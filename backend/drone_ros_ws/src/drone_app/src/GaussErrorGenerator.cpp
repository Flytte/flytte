#include "detector/gazebo/GaussErrorGenerator.hpp"

GaussErrorGenerator::GaussErrorGenerator(double mean, double std_dev) :
    m_generator(std::random_device{}()),
    m_mean(mean),
    m_std_dev(std_dev)
    {}

Pose GaussErrorGenerator::generatePose()
{
    Pose res;

    res.rotX = this->generateValue();
    res.rotY = this->generateValue();
    res.rotZ = this->generateValue();

    return res;
}

double GaussErrorGenerator::generateValue()
{
    std::normal_distribution<double> dist(m_mean, m_std_dev);

    return dist(this->m_generator);
}
