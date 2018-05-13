#include "detector/gazebo/StatisticalErrorGenerator.hpp"

StatisticalErrorGenerator::StatisticalErrorGenerator(int rate, double min, double max) :
    m_generator(std::random_device{}()),
    m_rate(rate),
    m_min(min),
    m_max(max)
    {}

Pose StatisticalErrorGenerator::generatePose()
{
    Pose res;

    std::uniform_int_distribution<int> dist(0, m_rate);

    if(dist(m_generator) == 0)
    {
        res.posX = this->generateValue();
        res.posY = this->generateValue();
        res.posZ = this->generateValue();
    }

    return res;
}

double StatisticalErrorGenerator::generateValue()
{
    std::uniform_int_distribution<int> dist(m_min, m_max);

    return dist(this->m_generator);
}
