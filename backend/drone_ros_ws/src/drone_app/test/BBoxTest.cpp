#include "common/BBox.hpp"

#include <gtest/gtest.h>

TEST(BBox, defaultConstructor)
{
    BBox box;

    EXPECT_EQ(0, box.posX);
    EXPECT_EQ(0, box.posY);
    EXPECT_EQ(0, box.w);
    EXPECT_EQ(0, box.h);
    EXPECT_EQ(0, box.rotX);
    EXPECT_EQ(0, box.rotY);
    EXPECT_EQ(0, box.rotZ);
}

TEST(BBox, copyConstructor)
{
    BBox box0(10, 20, 30, 40, 50, 60, 70);
    BBox box1(box0);

    EXPECT_EQ(10, box1.posX);
    EXPECT_EQ(20, box1.posY);
    EXPECT_EQ(30, box1.w);
    EXPECT_EQ(40, box1.h);
    EXPECT_EQ(50, box1.rotX);
    EXPECT_EQ(60, box1.rotY);
    EXPECT_EQ(70, box1.rotZ);
}

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
