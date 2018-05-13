#include "common/Pose.hpp"

#include <gtest/gtest.h>

TEST(Pose, defaultConstructor)
{
    Pose pose;

    EXPECT_EQ(0, pose.posX);
    EXPECT_EQ(0, pose.posY);
    EXPECT_EQ(0, pose.posZ);
    EXPECT_EQ(0, pose.rotX);
    EXPECT_EQ(0, pose.rotY);
    EXPECT_EQ(0, pose.rotZ);
}

TEST(Pose, copyConstructor)
{
    Pose pose0(10, 20, 30, 40, 50, 60);
    Pose pose1(pose0);

    EXPECT_EQ(10, pose1.posX);
    EXPECT_EQ(20, pose1.posY);
    EXPECT_EQ(30, pose1.posZ);
    EXPECT_EQ(40, pose1.rotX);
    EXPECT_EQ(50, pose1.rotY);
    EXPECT_EQ(60, pose1.rotZ);
}

TEST(Pose, additionAndAssignmentOperator)
{
    Pose pose0(10, 20, 30, 40, 50, 60);
    Pose pose1(10, 20, 30, 40, 50, 60);

    pose0 += pose1;

    EXPECT_EQ(20, pose0.posX);
    EXPECT_EQ(40, pose0.posY);
    EXPECT_EQ(60, pose0.posZ);
    EXPECT_EQ(80, pose0.rotX);
    EXPECT_EQ(100, pose0.rotY);
    EXPECT_EQ(120, pose0.rotZ);
}

TEST(Pose, additionOperator)
{
    Pose pose0(10, 20, 30, 40, 50, 60);
    Pose pose1(10, 20, 30, 40, 50, 60);

    Pose pose2 = pose0 + pose1;

    EXPECT_EQ(20, pose2.posX);
    EXPECT_EQ(40, pose2.posY);
    EXPECT_EQ(60, pose2.posZ);
    EXPECT_EQ(80, pose2.rotX);
    EXPECT_EQ(100, pose2.rotY);
    EXPECT_EQ(120, pose2.rotZ);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
