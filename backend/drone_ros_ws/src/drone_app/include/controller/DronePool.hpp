#ifndef STORAGE_HPP_
#define STORAGE_HPP_

#include <unordered_map>
#include <vector>
#include <functional>

#include "controller/Drone.hpp"

/**
 *  Centralized storage of the drones. Object of this class should be used
 *  to create, delete and store drones. Don't create Drone objects directly.
 */
// TODO Const safety
class DronePool
{
public:
    /**
     *  Default constructor.
     */
    DronePool();

    /**
     *  Getter for all the drones with their IDs.
     *  
     *  \return Vector of pairs of an ID and a reference to the drone
     */
    std::vector<std::pair<unsigned int, std::reference_wrapper<Drone>>> all();
    
    /**
     *  Getter for all the drones.
     *  
     *  \return Vector of references to the drones
     */
    std::vector<std::reference_wrapper<Drone>> drones();
    
    /**
     *  Getter for all the IDs of the drones.
     *  
     *  \return Vector of IDs of the drones
     */
    std::vector<unsigned int> keys();

    /**
     *  Looks for a drone.
     *  
     *  \param [in] id ID of the drone
     *  
     *  \throw std::out_of_range if the drone is not found
     *  
     *  \return Reference to the drone if found
     */
    Drone& findDrone(unsigned int id);
    
    /**
     *  Crates a new drone with the given data.
     *  
     *  \param [in] name Name (model) of the drone
     *  \param [in] box  Bounding box
     *  
     *  \return ID that was assigned to the drone
     */
    unsigned int createDrone(std::string name, BBox box);

    /**
     *  Indicates whether a drone with the given ID exists.
     *  
     *  \param [in] id ID of the drone
     *  
     *  \return True if the drone with the given ID exists,
     *          false otherwise
     */
    bool hasDrone(unsigned int id);
    
    /**
     *  Indicates the size of the storage, that is how many drones there are.
     *  
     *  \return Amount of the existing drones
     */
    unsigned int size();

    /**
     *  Destructor.
     */
    ~DronePool();

private:
    /**
     *  Next free ID of a drone.
     */
    unsigned int m_index;
    
    /**
     *  Hash map where the drones are stored.
     */
    std::unordered_map<unsigned int, Drone*> m_map;
};

#endif /* STORAGE_HPP_ */
