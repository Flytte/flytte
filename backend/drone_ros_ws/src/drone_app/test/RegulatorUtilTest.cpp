#include "controller/RegulatorUtil.hpp"
#include "controller/Math.hpp"

#include<ros/ros.h>
#include <gtest/gtest.h>

class RegulatorUtilTest : public ::testing::Test
{
protected:

    RegulatorUtilTest() :
        _d0("Dronych", 0, 0, 10, 10, 0, 0, 0),
        _ok_current(false),
        _ok_last(false),
        _ok_target(false) {}

    virtual void setBoxes()
    {
        std::tie(_box_current, _ok_current) = _d0.getBBox(Drone::BBoxKind::CURRENT);
        std::tie(_box_last, _ok_last) = _d0.getBBox(Drone::BBoxKind::LAST);
        std::tie(_box_target, _ok_target) = _d0.getBBox(Drone::BBoxKind::TARGET);
    }

    Math _math;
    Drone _d0;

    BBox _box_current;
    BBox _box_last;
    BBox _box_target;

    bool _ok_current;
    bool _ok_last;
    bool _ok_target;
};


TEST_F(RegulatorUtilTest, noTargetNorLastSet)
{
    RegulatorResult res;

    ASSERT_NO_THROW(res = RegulatorUtil::checkState(_math, _d0));

    EXPECT_EQ(RegulatorResult::ANOMALY, res);
}

TEST_F(RegulatorUtilTest, noTargetSet)
{
    _d0.setCurrent(BBox(10, 10, 10, 0, 0, 0, 0));

    RegulatorResult res;

    ASSERT_NO_THROW(res = RegulatorUtil::checkState(_math, _d0));

    EXPECT_EQ(RegulatorResult::ANOMALY, res);
}

TEST_F(RegulatorUtilTest, noLastSet)
{
    _d0.setTarget(BBox(10, 10, 10, 0, 0, 0, 0));

    RegulatorResult res;

    ASSERT_NO_THROW(res = RegulatorUtil::checkState(_math, _d0));

    EXPECT_EQ(RegulatorResult::ANOMALY, res);
}

TEST_F(RegulatorUtilTest, zeroSizedBBoxes)
{
    _d0.setCurrent(BBox(10, 10, 0, 0, 0, 0, 0));
    _d0.setTarget(BBox(10, 10, 0, 0, 0, 0, 0));

    RegulatorResult res;

    ASSERT_NO_THROW(res = RegulatorUtil::checkState(_math, _d0));

    EXPECT_EQ(RegulatorResult::ANOMALY, res);
}

TEST_F(RegulatorUtilTest, arrivedExactly)
{
    _d0.setCurrent(BBox(10, 10, 10, 10, 0, 0, 0));
    _d0.setTarget(BBox(10, 10, 10, 10, 0, 0, 0));

    RegulatorResult res;

    ASSERT_NO_THROW(res = RegulatorUtil::checkState(_math, _d0));

    EXPECT_EQ(RegulatorResult::ARRIVED, res);
}

TEST_F(RegulatorUtilTest, arrivedApproximately_XY)
{
    _d0.setCurrent(BBox(49, 50, 10, 10, 0, 0, 0));
    _d0.setTarget(BBox(50, 50, 10, 10, 0, 0, 0));

    RegulatorResult res;

    ASSERT_NO_THROW(res = RegulatorUtil::checkState(_math, _d0));

    EXPECT_EQ(RegulatorResult::ARRIVED, res);
}

TEST_F(RegulatorUtilTest, arrivedApproximately_Z)
{
    _d0.setCurrent(BBox(50, 50, 24, 24, 0, 0, 0));
    _d0.setTarget(BBox(50, 50, 25, 25, 0, 0, 0));

    RegulatorResult res;

    ASSERT_NO_THROW(res = RegulatorUtil::checkState(_math, _d0));

    EXPECT_EQ(RegulatorResult::ARRIVED, res);
}

TEST_F(RegulatorUtilTest, arrivedApproximately_XYZ)
{
    _d0.setCurrent(BBox(49, 51, 24, 24, 0, 0, 0));
    _d0.setTarget(BBox(50, 50, 25, 25, 0, 0, 0));

    RegulatorResult res;

    ASSERT_NO_THROW(res = RegulatorUtil::checkState(_math, _d0));

    EXPECT_EQ(RegulatorResult::ARRIVED, res);
}

TEST_F(RegulatorUtilTest, toTheSide_Y)
{
    _d0.setCurrent(BBox(0, 25, 10, 10, 0, 0, 0));
    _d0.setTarget(BBox(50, 50, 10, 10, 0, 0, 0));

    RegulatorResult res;

    ASSERT_NO_THROW(res = RegulatorUtil::checkState(_math, _d0));

    EXPECT_EQ(RegulatorResult::WRONG_DIRECTION, res);
}

TEST_F(RegulatorUtilTest, toTheSide_X)
{
    _d0.setCurrent(BBox(25, 0, 10, 10, 0, 0, 0));
    _d0.setTarget(BBox(50, 50, 10, 10, 0, 0, 0));

    RegulatorResult res;

    ASSERT_NO_THROW(res = RegulatorUtil::checkState(_math, _d0));

    EXPECT_EQ(RegulatorResult::WRONG_DIRECTION, res);
}

TEST_F(RegulatorUtilTest, toTheSide_Z)
{
    _d0.setCurrent(BBox(0, 0, 17, 17, 0, 0, 0));
    _d0.setTarget(BBox(50, 50, 35, 35, 0, 0, 0));

    RegulatorResult res;

    ASSERT_NO_THROW(res = RegulatorUtil::checkState(_math, _d0));

    EXPECT_EQ(RegulatorResult::WRONG_DIRECTION, res);
}

TEST_F(RegulatorUtilTest, toTheSide_XY)
{
    _d0.setCurrent(BBox(25, 25, 10, 10, 0, 0, 0));
    _d0.setTarget(BBox(50, 50, 35, 35, 0, 0, 0));

    RegulatorResult res;

    ASSERT_NO_THROW(res = RegulatorUtil::checkState(_math, _d0));

    EXPECT_EQ(RegulatorResult::WRONG_DIRECTION, res);
}

TEST_F(RegulatorUtilTest, toTheSide_XZ)
{
    _d0.setCurrent(BBox(25, 0, 17, 17, 0, 0, 0));
    _d0.setTarget(BBox(50, 50, 35, 35, 0, 0, 0));

    RegulatorResult res;

    ASSERT_NO_THROW(res = RegulatorUtil::checkState(_math, _d0));

    EXPECT_EQ(RegulatorResult::WRONG_DIRECTION, res);
}

TEST_F(RegulatorUtilTest, toTheSide_YZ)
{
    _d0.setCurrent(BBox(0, 25, 17, 17, 0, 0, 0));
    _d0.setTarget(BBox(50, 50, 35, 35, 0, 0, 0));

    RegulatorResult res;

    ASSERT_NO_THROW(res = RegulatorUtil::checkState(_math, _d0));

    EXPECT_EQ(RegulatorResult::WRONG_DIRECTION, res);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "RegulatorUtilTest");

    ros::param::set("/drones/Dronych/size/width", 382.0);
    ros::param::set("/drones/Dronych/size/height", 89.0);
    ros::param::set("/drones/Dronych/size/depth", 328.0);

    ros::param::set("/camera/w", 640);
    ros::param::set("/camera/h", 480);

    ros::param::set("/drone_velocity/max_h", 32);
    ros::param::set("/drone_velocity/max_v", 5);

    return RUN_ALL_TESTS();
}
