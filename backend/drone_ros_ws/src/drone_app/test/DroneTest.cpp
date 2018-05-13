#include "controller/Drone.hpp"

#include <gtest/gtest.h>

class DroneTest : public ::testing::Test
{
protected:

    DroneTest() :
        _d0("Dronych", 10, 20, 30, 40, 50, 60, 70),
        _ok_current(false),
        _ok_last(false),
        _ok_target(false) {}

    virtual void setBoxes()
    {
        std::tie(_box_current, _ok_current) = _d0.getBBox(Drone::BBoxKind::CURRENT);
        std::tie(_box_last, _ok_last) = _d0.getBBox(Drone::BBoxKind::LAST);
        std::tie(_box_target, _ok_target) = _d0.getBBox(Drone::BBoxKind::TARGET);
    }

    Drone _d0;

    BBox _box_current;
    BBox _box_last;
    BBox _box_target;

    bool _ok_current;
    bool _ok_last;
    bool _ok_target;
};


TEST_F(DroneTest, constructorDefaultValues)
{
    Drone d1("Dronych");

    BBox box_current, box_last, box_target;
    bool ok_current, ok_last, ok_target;

    std::tie(box_current, ok_current) = d1.getBBox(Drone::BBoxKind::CURRENT);
    std::tie(box_last, ok_last) = d1.getBBox(Drone::BBoxKind::LAST);
    std::tie(box_target, ok_target) = d1.getBBox(Drone::BBoxKind::TARGET);

    EXPECT_TRUE(ok_current);
    EXPECT_FALSE(ok_last);
    EXPECT_FALSE(ok_target);

    EXPECT_EQ("Dronych", d1.name());

    EXPECT_EQ(0, box_current.posX);
    EXPECT_EQ(0, box_current.posY);
    EXPECT_EQ(0, box_current.w);
    EXPECT_EQ(0, box_current.h);
    EXPECT_EQ(0, box_current.rotX);
    EXPECT_EQ(0, box_current.rotY);
    EXPECT_EQ(0, box_current.rotZ);

    EXPECT_EQ(0, box_last.posX);
    EXPECT_EQ(0, box_last.posY);
    EXPECT_EQ(0, box_last.w);
    EXPECT_EQ(0, box_last.h);
    EXPECT_EQ(0, box_last.rotX);
    EXPECT_EQ(0, box_last.rotY);
    EXPECT_EQ(0, box_last.rotZ);

    EXPECT_EQ(0, box_target.posX);
    EXPECT_EQ(0, box_target.posY);
    EXPECT_EQ(0, box_target.w);
    EXPECT_EQ(0, box_target.h);
    EXPECT_EQ(0, box_target.rotX);
    EXPECT_EQ(0, box_target.rotY);
    EXPECT_EQ(0, box_target.rotZ);

    EXPECT_FALSE(d1.hasTarget());
}

TEST_F(DroneTest, afterCreation)
{
    setBoxes();

    EXPECT_TRUE(_ok_current);
    EXPECT_FALSE(_ok_last);
    EXPECT_FALSE(_ok_target);

    EXPECT_EQ(10, _box_current.posX);
    EXPECT_EQ(20, _box_current.posY);
    EXPECT_EQ(30, _box_current.w);
    EXPECT_EQ(40, _box_current.h);
    EXPECT_EQ(50, _box_current.rotX);
    EXPECT_EQ(60, _box_current.rotY);
    EXPECT_EQ(70, _box_current.rotZ);

    EXPECT_EQ(0, _box_last.posX);
    EXPECT_EQ(0, _box_last.posY);
    EXPECT_EQ(0, _box_last.w);
    EXPECT_EQ(0, _box_last.h);
    EXPECT_EQ(0, _box_last.rotX);
    EXPECT_EQ(0, _box_last.rotY);
    EXPECT_EQ(0, _box_last.rotZ);

    EXPECT_EQ(0, _box_target.posX);
    EXPECT_EQ(0, _box_target.posY);
    EXPECT_EQ(0, _box_target.w);
    EXPECT_EQ(0, _box_target.h);
    EXPECT_EQ(0, _box_target.rotX);
    EXPECT_EQ(0, _box_target.rotY);
    EXPECT_EQ(0, _box_target.rotZ);

    EXPECT_FALSE(_d0.hasTarget());
}

TEST_F(DroneTest, afterSetCurrent)
{
    _d0.setCurrent(BBox(15, 25, 35, 45, 55, 65, 75));

    setBoxes();

    EXPECT_TRUE(_ok_current);
    EXPECT_TRUE(_ok_last);
    EXPECT_FALSE(_ok_target);

    EXPECT_EQ(15, _box_current.posX);
    EXPECT_EQ(25, _box_current.posY);
    EXPECT_EQ(35, _box_current.w);
    EXPECT_EQ(45, _box_current.h);
    EXPECT_EQ(55, _box_current.rotX);
    EXPECT_EQ(65, _box_current.rotY);
    EXPECT_EQ(75, _box_current.rotZ);

    EXPECT_EQ(10, _box_last.posX);
    EXPECT_EQ(20, _box_last.posY);
    EXPECT_EQ(30, _box_last.w);
    EXPECT_EQ(40, _box_last.h);
    EXPECT_EQ(50, _box_last.rotX);
    EXPECT_EQ(60, _box_last.rotY);
    EXPECT_EQ(70, _box_last.rotZ);

    EXPECT_EQ(0, _box_target.posX);
    EXPECT_EQ(0, _box_target.posY);
    EXPECT_EQ(0, _box_target.w);
    EXPECT_EQ(0, _box_target.h);
    EXPECT_EQ(0, _box_target.rotX);
    EXPECT_EQ(0, _box_target.rotY);
    EXPECT_EQ(0, _box_target.rotZ);

    EXPECT_FALSE(_d0.hasTarget());
}

TEST_F(DroneTest, afterSetTarget)
{
    _d0.setTarget(BBox(15, 25, 35, 45, 55, 65, 75));

    setBoxes();

    EXPECT_TRUE(_ok_current);
    EXPECT_FALSE(_ok_last);
    EXPECT_TRUE(_ok_target);

    EXPECT_EQ(10, _box_current.posX);
    EXPECT_EQ(20, _box_current.posY);
    EXPECT_EQ(30, _box_current.w);
    EXPECT_EQ(40, _box_current.h);
    EXPECT_EQ(50, _box_current.rotX);
    EXPECT_EQ(60, _box_current.rotY);
    EXPECT_EQ(70, _box_current.rotZ);

    EXPECT_EQ(0, _box_last.posX);
    EXPECT_EQ(0, _box_last.posY);
    EXPECT_EQ(0, _box_last.w);
    EXPECT_EQ(0, _box_last.h);
    EXPECT_EQ(0, _box_last.rotX);
    EXPECT_EQ(0, _box_last.rotY);
    EXPECT_EQ(0, _box_last.rotZ);

    EXPECT_EQ(15, _box_target.posX);
    EXPECT_EQ(25, _box_target.posY);
    EXPECT_EQ(35, _box_target.w);
    EXPECT_EQ(45, _box_target.h);
    EXPECT_EQ(55, _box_target.rotX);
    EXPECT_EQ(65, _box_target.rotY);
    EXPECT_EQ(75, _box_target.rotZ);

    EXPECT_TRUE(_d0.hasTarget());
}

TEST_F(DroneTest, afterClearTarget)
{
    _d0.setTarget(BBox(15, 25, 35, 45, 55, 65, 75));
    _d0.clearTarget();

    setBoxes();

    EXPECT_TRUE(_ok_current);
    EXPECT_FALSE(_ok_last);
    EXPECT_FALSE(_ok_target);

    EXPECT_EQ(10, _box_current.posX);
    EXPECT_EQ(20, _box_current.posY);
    EXPECT_EQ(30, _box_current.w);
    EXPECT_EQ(40, _box_current.h);
    EXPECT_EQ(50, _box_current.rotX);
    EXPECT_EQ(60, _box_current.rotY);
    EXPECT_EQ(70, _box_current.rotZ);

    EXPECT_EQ(0, _box_last.posX);
    EXPECT_EQ(0, _box_last.posY);
    EXPECT_EQ(0, _box_last.w);
    EXPECT_EQ(0, _box_last.h);
    EXPECT_EQ(0, _box_last.rotX);
    EXPECT_EQ(0, _box_last.rotY);
    EXPECT_EQ(0, _box_last.rotZ);

    EXPECT_EQ(0, _box_target.posX);
    EXPECT_EQ(0, _box_target.posY);
    EXPECT_EQ(0, _box_target.w);
    EXPECT_EQ(0, _box_target.h);
    EXPECT_EQ(0, _box_target.rotX);
    EXPECT_EQ(0, _box_target.rotY);
    EXPECT_EQ(0, _box_target.rotZ);

    EXPECT_FALSE(_d0.hasTarget());
}

TEST_F(DroneTest, copyConstructor)
{
    BBox box_current, box_last, box_target;
    bool ok_current, ok_last, ok_target;

    _d0.setTarget(BBox(15, 25, 35, 45, 55, 65, 75));

    Drone drone(_d0);

    std::tie(box_current, ok_current) = drone.getBBox(Drone::BBoxKind::CURRENT);
    std::tie(box_last, ok_last) = drone.getBBox(Drone::BBoxKind::LAST);
    std::tie(box_target, ok_target) = drone.getBBox(Drone::BBoxKind::TARGET);

    EXPECT_TRUE(ok_current);
    EXPECT_FALSE(ok_last);
    EXPECT_TRUE(ok_target);

    EXPECT_EQ("Dronych", drone.name());

    EXPECT_EQ(10, box_current.posX);
    EXPECT_EQ(20, box_current.posY);
    EXPECT_EQ(30, box_current.w);
    EXPECT_EQ(40, box_current.h);
    EXPECT_EQ(50, box_current.rotX);
    EXPECT_EQ(60, box_current.rotY);
    EXPECT_EQ(70, box_current.rotZ);

    EXPECT_EQ(0, box_last.posX);
    EXPECT_EQ(0, box_last.posY);
    EXPECT_EQ(0, box_last.w);
    EXPECT_EQ(0, box_last.h);
    EXPECT_EQ(0, box_last.rotX);
    EXPECT_EQ(0, box_last.rotY);
    EXPECT_EQ(0, box_last.rotZ);

    EXPECT_EQ(15, box_target.posX);
    EXPECT_EQ(25, box_target.posY);
    EXPECT_EQ(35, box_target.w);
    EXPECT_EQ(45, box_target.h);
    EXPECT_EQ(55, box_target.rotX);
    EXPECT_EQ(65, box_target.rotY);
    EXPECT_EQ(75, box_target.rotZ);

    EXPECT_TRUE(drone.hasTarget());
}

TEST_F(DroneTest, assignmentOperator)
{
    BBox box_current, box_last, box_target;
    bool ok_current, ok_last, ok_target;

    _d0.setTarget(BBox(15, 25, 35, 45, 55, 65, 75));

    Drone drone("NotDronych");
    drone.setCurrent(BBox(1, 2, 3, 4, 5, 6, 7));

    drone = _d0;

    std::tie(box_current, ok_current) = drone.getBBox(Drone::BBoxKind::CURRENT);
    std::tie(box_last, ok_last) = drone.getBBox(Drone::BBoxKind::LAST);
    std::tie(box_target, ok_target) = drone.getBBox(Drone::BBoxKind::TARGET);

    EXPECT_TRUE(ok_current);
    EXPECT_FALSE(ok_last);
    EXPECT_TRUE(ok_target);

    EXPECT_EQ("Dronych", drone.name());

    EXPECT_EQ(10, box_current.posX);
    EXPECT_EQ(20, box_current.posY);
    EXPECT_EQ(30, box_current.w);
    EXPECT_EQ(40, box_current.h);
    EXPECT_EQ(50, box_current.rotX);
    EXPECT_EQ(60, box_current.rotY);
    EXPECT_EQ(70, box_current.rotZ);

    EXPECT_EQ(0, box_last.posX);
    EXPECT_EQ(0, box_last.posY);
    EXPECT_EQ(0, box_last.w);
    EXPECT_EQ(0, box_last.h);
    EXPECT_EQ(0, box_last.rotX);
    EXPECT_EQ(0, box_last.rotY);
    EXPECT_EQ(0, box_last.rotZ);

    EXPECT_EQ(15, box_target.posX);
    EXPECT_EQ(25, box_target.posY);
    EXPECT_EQ(35, box_target.w);
    EXPECT_EQ(45, box_target.h);
    EXPECT_EQ(55, box_target.rotX);
    EXPECT_EQ(65, box_target.rotY);
    EXPECT_EQ(75, box_target.rotZ);

    EXPECT_TRUE(drone.hasTarget());
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
