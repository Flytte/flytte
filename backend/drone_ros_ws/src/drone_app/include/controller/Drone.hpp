#ifndef DRONE_HPP_
#define DRONE_HPP_

#include "common/BBox.hpp"
#include "common/Pose.hpp"

#include <string>

/**
 *  Stores information about a drone. Objects of this class
 *  have to be created and managed by a DronePool and not directly.
 */
class Drone
{
public:
    /**
     *  Identifies the art of bounding box stored by a drone object.
     */
    enum class BBoxKind
    {
        LAST,
        CURRENT,
        TARGET
    };

    /**
     *  Constructor.
     *  
     *  \param [in] name Name of the drone (its model)
     *  \param [in] posX Position x of the bounding box
     *  \param [in] posY Position y of the bounding box
     *  \param [in] w    Width of the bounding box
     *  \param [in] h    Height of the bounding box
     *  \param [in] rotX Rotation around x-axis
     *  \param [in] rotY Rotation around y-axis
     *  \param [in] rotZ Rotation around z-axis
     */
    Drone(std::string name, double posX = 0, double posY = 0, double w = 0,
            double h = 0, double rotX = 0, double rotY = 0, double rotZ = 0);
    
    /**
     *  Constructor.
     *  
     *  \param [in] name Name of the drone (its model)
     *  \param [in] box  Bounding box
     */
    Drone(std::string name, BBox box);

    /**
     *  Copy constructor.
     *  
     *  \param [in] other Drone to be copied
     */
    Drone(const Drone& other);

    /**
     *  Assignment operator.
     *  
     *  \param [in] other Drone to be copied
     */
    Drone& operator=(const Drone& other);

    /**
     *  Getter for the name (model) of the drone.
     *  
     *  \return Name (model) of the drone
     */
    std::string name() const {return m_name;}

    /**
     *  Getter for the current bounding box.
     *  
     *  \return Pair with the bounding box and a boolean
     *          identifying if the requested box exists
     *          (always true here)
     */
    std::pair<BBox, bool> current() const;
    
    /**
     *  Getter for the bounding box of previous position.
     *  
     *  \return Pair with the bounding box and a boolean
     *          identifying if the requested box exists
     *          (false if the drone is new and has not been moved yet),
     *          if second value is false the returned box has default values
     */
    std::pair<BBox, bool> last() const;
    
    /**
     *  Getter for the bounding box of target position.
     *  
     *  \return Pair with the bounding box and a boolean
     *          identifying if the requested box exists
     *          (false if target is not set),
     *          if second value is false the returned box has default values
     */
    std::pair<BBox, bool> target() const;

    /**
     *  Generic getter of a bounding box stored by the drone.
     *  
     *  \param [in] id Art of the bounding box to be returned
     *  
     *  \return Pair with the bounding box and a boolean
     *          identifying if the requested box exists
     *          (false if target is not set),
     *          if second value is false the returned box has default values
     */
    std::pair<BBox, bool> getBBox(Drone::BBoxKind id) const;

    /**
     *  Setter for the current bounding box.
     *  
     *  \param [in] box Bounding box to be set
     */
    void setCurrent(const BBox box);
    
    /**
     *  Setter for the target bounding box.
     *  
     *  \param [in] box Bounding box to be set
     */
    void setTarget(const BBox box);
    
    /**
     *  Clears the target bounding box.
     */
    void clearTarget();

    /**
     *  Indicates whether a target bounding box is set.
     *  
     *  \return True if target bounding box is set,
     *          false otherwise
     */
    bool hasTarget() const {return m_target != nullptr;}

    /**
     *  Destructor.
     */
    ~Drone();

private:
    /**
     *  Name (model) of the drone.
     */
    std::string m_name;
    
    /**
     *  Bounding box of the target pose.
     */
    BBox* m_current;
    
    /**
     *  Bounding box of the previous pose.
     */
    BBox* m_last;
    
    /**
     *  Bounding box of the target pose.
     */
    BBox* m_target;
};

#endif /* DRONE_HPP_ */
