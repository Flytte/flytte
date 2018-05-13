#include "controller/DronePool.hpp"

#include <stdexcept>

typedef std::unordered_map<unsigned int, Drone*>::iterator Iterator;

DronePool::DronePool() : m_index(0) {}

std::vector<std::pair<unsigned int, std::reference_wrapper<Drone>>> DronePool::all()
{
    std::vector<std::pair<unsigned int, std::reference_wrapper<Drone>>> res;
    for(auto pair : m_map)
    {
        res.push_back(
            std::pair<unsigned int, std::reference_wrapper<Drone>>(
                pair.first, std::ref(*(pair.second))));
    }

    return res;
}

std::vector<std::reference_wrapper<Drone>> DronePool::drones()
{
    std::vector<std::reference_wrapper<Drone>> res;
    for(auto pair : m_map)
    {
        res.push_back(std::ref(*(pair.second)));
    }

    return res;
}

Drone& DronePool::findDrone(unsigned int id)
{
    return *(m_map.at(id));
}

unsigned int DronePool::createDrone(std::string name, BBox box)
{
    std::pair<Iterator, bool> res;
    res = m_map.insert(std::make_pair(m_index++, new Drone(name, box)));

    if(!std::get<1>(res))
    {
        throw std::invalid_argument("Adding the drone failed");
    }

    return std::get<0>(res)->first;
}

bool DronePool::hasDrone(unsigned int id)
{
    return m_map.count(id) != 0;
}

unsigned int DronePool::size()
{
    return m_map.size();
}

std::vector<unsigned int> DronePool::keys()
{
    std::vector<unsigned int> res;
    for(auto pair : m_map)
    {
        res.push_back(pair.first);
    }

    return res;
}

DronePool::~DronePool()
{
    for(auto pair : m_map)
    {
        delete pair.second;
    }

    m_map.clear();
}
