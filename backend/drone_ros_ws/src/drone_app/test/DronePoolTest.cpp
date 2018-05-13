#include "controller/DronePool.hpp"

#include <gtest/gtest.h>

TEST(DronePool, afterCreation)
{
    DronePool s;

    EXPECT_EQ(0, s.size());
    EXPECT_EQ(0, s.all().size());
    EXPECT_EQ(0, s.drones().size());
    EXPECT_EQ(0, s.keys().size());
    EXPECT_FALSE(s.hasDrone(0));

    try
    {
        s.findDrone(0);
    }
    catch(std::out_of_range e)
    {
        SUCCEED();
    }
    catch(...)
    {
        ADD_FAILURE() << "Unexpected exception";
    }
}

TEST(DronePool, afterAddDrone)
{
    DronePool s;
    BBox box_current;
    bool ok;

    s.createDrone("Dronych", BBox(10, 20, 30, 40, 50, 60, 70));

    EXPECT_EQ(1, s.size());
    EXPECT_EQ(1, s.all().size());
    EXPECT_EQ(1, s.drones().size());
    EXPECT_EQ(1, s.keys().size());
    EXPECT_TRUE(s.hasDrone(0));
    EXPECT_FALSE(s.hasDrone(1));

    try
    {
        std::tie(box_current, ok) = s.findDrone(0).current();

        EXPECT_EQ(0, s.all().at(0).first);
        EXPECT_EQ(0, s.keys().at(0));
    }
    catch(std::out_of_range e)
    {
        ADD_FAILURE() << "out_of_range exception";
    }
    catch(...)
    {
        ADD_FAILURE() << "Unexpected exception";
    }

    EXPECT_EQ(10, box_current.posX);
    EXPECT_EQ(20, box_current.posY);
    EXPECT_EQ(30, box_current.w);
    EXPECT_EQ(40, box_current.h);
    EXPECT_EQ(50, box_current.rotX);
    EXPECT_EQ(60, box_current.rotY);
    EXPECT_EQ(70, box_current.rotZ);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}