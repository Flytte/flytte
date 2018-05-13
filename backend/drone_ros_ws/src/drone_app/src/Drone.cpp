#include "controller/Drone.hpp"

#include <iostream>

Drone::Drone(std::string name, double posX, double posY, double w, double h,
        double rotX, double rotY, double rotZ) :
    m_name(name),
    m_current(new BBox(posX, posY, w, h, rotX, rotY, rotZ)),
    m_last(nullptr),
    m_target(nullptr) {}

Drone::Drone(std::string name, BBox box) :
    m_name(name),
    m_current(new BBox(box)),
    m_last(nullptr),
    m_target(nullptr) {}

Drone::Drone(const Drone& other) :
    m_name(other.name()),
    m_current(new BBox(other.current().first)),
    m_last(nullptr),
    m_target(nullptr)
{
    if(other.last().second)
    {
        this->m_last = new BBox(other.last().first);
    }

    if(other.target().second)
    {
        this->m_target = new BBox(other.target().first);
    }
}

Drone& Drone::operator=(const Drone& other)
{
    m_name = other.name();

    *m_current = other.current().first;

    if(other.last().second)
    {
        if(!this->m_last)
        {
            this->m_last = new BBox;
        }

        *(this->m_last) = BBox(other.last().first);
    }
    else
    {
        if(this->m_last)
        {
            delete this->m_last;
            this->m_last = nullptr;
        }
    }

    if(other.target().second)
    {
        if(!this->m_target)
        {
            this->m_target = new BBox;
        }

        *(this->m_target) = BBox(other.target().first);
    }
    else
    {
        if(this->m_target)
        {
            delete this->m_target;
            this->m_target = nullptr;
        }
    }

    return *this;
}

std::pair<BBox, bool> Drone::current() const
{
    return std::make_pair(*m_current, true);
}

std::pair<BBox, bool> Drone::last() const
{
    if(!m_last)
    {
        return std::make_pair(BBox(), false);
    }

    return std::make_pair(*m_last, true);
}

std::pair<BBox, bool> Drone::target() const
{
    if(!m_target)
    {
        return std::make_pair(BBox(), false);
    }

    return std::make_pair(*m_target, true);
}

std::pair<BBox, bool> Drone::getBBox(Drone::BBoxKind id) const
{
    if(id == Drone::BBoxKind::LAST)
    {
        return this->last();
    }
    else if(id == Drone::BBoxKind::TARGET)
    {
        return this->target();
    }
    else
    {
        return this->current();
    }
}

void Drone::setCurrent(BBox box)
{
    if(!m_last)
    {
        m_last = new BBox();
    }

    *m_last = *m_current;
    *m_current = box;
}

void Drone::setTarget(BBox box)
{
    if(!m_target)
    {
        m_target = new BBox();
    }

    *m_target = box;
}

void Drone::clearTarget()
{
    if(m_target != nullptr)
    {
        delete m_target;
        m_target = nullptr;
    }
}

Drone::~Drone()
{
    delete this->m_current;

    if(!this->m_last)
    {
        delete this->m_last;
        this->m_last = nullptr;
    }

    if(!this->m_target)
    {
        delete this->m_target;
        this->m_target = nullptr;
    }
}