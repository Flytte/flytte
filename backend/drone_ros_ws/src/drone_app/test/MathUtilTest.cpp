#include "controller/Math.hpp"
#include "controller/Drone.hpp"
#include "ros/ros.h"
#include <gtest/gtest.h>

TEST( MathTest, directionVector)
{
  Pose current_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  Pose target_pose(0.0, 0.0, 100.0, 0.0, 0.0, 0.0);

  Math* math = new Math();
  Pose result = math->directionVector( current_pose, target_pose);

  EXPECT_DOUBLE_EQ(0.0, result.posX);
  EXPECT_DOUBLE_EQ(0.0, result.posY);
  EXPECT_DOUBLE_EQ(100.0, result.posZ);
  EXPECT_DOUBLE_EQ(0.0, result.rotZ);
}

TEST( MathTest, normalize)
{
  Pose vector;
  vector.posX = 10.0;
  vector.posY = 5.0;
  vector.posZ = 0.0;
  vector.rotZ = 180.0;

  Math* math = new Math();
  Pose result = math->normalize( vector);

  EXPECT_GE(result.posX, -1.0);
  EXPECT_LE(result.posX, 1.0);
  EXPECT_GE(result.posY, -1.0);
  EXPECT_LE(result.posY, 1.0);
  EXPECT_GE(result.posZ, -1.0);
  EXPECT_LE(result.posZ, 1.0);
}

TEST( MathTest, rotate2ClientSimulationBasis)
{
  ros::param::set("/camera/x", 0.0);
  ros::param::set("/camera/y", 0.0);
  ros::param::set("/camera/z", 0.0);

  // --- Easy test, no rotation of the drone
  // Rotate Ox
  Pose current_pose(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  Math* math = new Math();
  Pose result = math->rotate2ClientSimulation(current_pose);

  EXPECT_EQ(0.0, result.posX);
  EXPECT_EQ(0.0, result.posY);
  EXPECT_EQ(1.0, result.posZ);
  EXPECT_EQ(0.0, result.rotZ);

  // Rotate Oy
  current_pose.posX = 0.0;
  current_pose.posY = 1.0;
  current_pose.posZ = 0.0;

  result = math->rotate2ClientSimulation(current_pose);

  EXPECT_EQ(-1.0, result.posX);
  EXPECT_EQ(0.0, result.posY);
  EXPECT_EQ(0.0, result.posZ);
  EXPECT_EQ(0.0, result.rotZ);

  // Rotate Oz
  current_pose.posX = 0.0;
  current_pose.posY = 0.0;
  current_pose.posZ = 1.0;

  result = math->rotate2ClientSimulation(current_pose);

  EXPECT_EQ(0.0, result.posX);
  EXPECT_EQ(-1.0, result.posY);
  EXPECT_EQ(0.0, result.posZ);
  EXPECT_EQ(0.0, result.rotZ);
}

TEST( MathTest, rotate2ClientSimulationVector)
{
  ros::param::set("/camera/x", 0.0);
  ros::param::set("/camera/y", 0.0);
  ros::param::set("/camera/z", 0.0);

  // --- Easy test, no rotation of the drone
  Pose current_pose(1.0, 2.0, 3.0, 0.0, 0.0, 0.0);

  Math* math = new Math();
  Pose result = math->rotate2ClientSimulation(current_pose);

  EXPECT_EQ(-2.0, result.posX);
  EXPECT_EQ(-3.0, result.posY);
  EXPECT_EQ(1.0, result.posZ);
  EXPECT_EQ(0.0, result.rotZ);
}

TEST( MathTest, rotate2ClientSimulationMoveOz)
{
  ros::param::set("/camera/x", 0.0);
  ros::param::set("/camera/y", 0.0);
  ros::param::set("/camera/z", 0.0);

  // --- Easy test, no rotation of the drone
  // Rotate Ox
  Pose current_pose(10.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  Math* math = new Math();
  Pose result = math->rotate2ClientSimulation(current_pose);

  EXPECT_EQ(0.0, result.posX);
  EXPECT_EQ(0.0, result.posY);
  EXPECT_EQ(10.0, result.posZ);
  EXPECT_EQ(0.0, result.rotZ);

  current_pose.posX += 10.;
  result = math->rotate2ClientSimulation(current_pose);

  EXPECT_EQ(0.0, result.posX);
  EXPECT_EQ(0.0, result.posY);
  EXPECT_EQ(20.0, result.posZ);
  EXPECT_EQ(0.0, result.rotZ);

  current_pose.posX += 10.;
  result = math->rotate2ClientSimulation(current_pose);

  EXPECT_EQ(0.0, result.posX);
  EXPECT_EQ(0.0, result.posY);
  EXPECT_EQ(30.0, result.posZ);
  EXPECT_EQ(0.0, result.rotZ);
}

TEST( MathTest, rotate2ClientMoveCamera)
{
  ros::param::set("/camera/x", -0.5);
  ros::param::set("/camera/y", 0.0);
  ros::param::set("/camera/z", 0.0);

  // --- Easy test, no rotation of the drone
  Pose current_pose(0.5, 0.0, 0.0, 0.0, 0.0, 0.0);

  Math* math = new Math();
  Pose result = math->rotate2ClientSimulation(current_pose);

  EXPECT_EQ(0.0, result.posX);
  EXPECT_EQ(0.0, result.posY);
  EXPECT_EQ(1.0, result.posZ);
  EXPECT_EQ(0.0, result.rotZ);
}


TEST( MathTest, pose2BBoxMoveOx)
{
  ros::param::set("/camera/d1", 0.0);
  ros::param::set("/camera/d2", 0.0);
  ros::param::set("/camera/d3", 0.0);
  ros::param::set("/camera/d4", 0.0);
  ros::param::set("/camera/d5", 0.0);

  Drone dr("dronych");

  Math* math = new Math();
  BBox bb1 = math->pose2BBox(dr, Pose(0.1, 0.0, 1.0, 0.0, 0.0, 0.0));
  BBox bb2 = math->pose2BBox(dr, Pose(0.9, 0.0, 1.0, 0.0, 0.0, 0.0));

  EXPECT_LE(bb1.posX, bb2.posX);
  EXPECT_NEAR(bb1.posY, bb2.posY, 0.01);
  EXPECT_NEAR(bb1.w, bb2.w, 0.01);
  EXPECT_NEAR(bb1.h, bb2.h, 0.01);
}

TEST( MathTest, pose2BBoxMoveOy)
{
  ros::param::set("/camera/d1", 0.0);
  ros::param::set("/camera/d2", 0.0);
  ros::param::set("/camera/d3", 0.0);
  ros::param::set("/camera/d4", 0.0);
  ros::param::set("/camera/d5", 0.0);

  Drone dr("dronych");

  Math* math = new Math();
  BBox bb1 = math->pose2BBox(dr, Pose(0.0, 0.1, 1.0, 0.0, 0.0, 0.0));
  BBox bb2 = math->pose2BBox(dr, Pose(0.0, 0.9, 1.0, 0.0, 0.0, 0.0));

  EXPECT_LE(bb1.posY, bb2.posY);
  EXPECT_NEAR(bb1.posX, bb2.posX,0.01);
  EXPECT_NEAR(bb1.w, bb2.w, 0.01);
  EXPECT_NEAR(bb1.h, bb2.h, 0.01);
}

TEST( MathTest, pose2BBoxMoveOz)
{
  ros::param::set("/camera/d1", 0.0);
  ros::param::set("/camera/d2", 0.0);
  ros::param::set("/camera/d3", 0.0);
  ros::param::set("/camera/d4", 0.0);
  ros::param::set("/camera/d5", 0.0);

  Drone dr("dronych");

  ros::param::set("/camera/x", 0.0);
  ros::param::set("/camera/y", 0.0);
  ros::param::set("/camera/z", 0.0);

  Math* math = new Math();
  BBox bb1 = math->pose2BBox(dr, Pose(0.0, 0.0, 1.0, 0.0, 0.0, 0.0));
  BBox bb2 = math->pose2BBox(dr, Pose(0.0, 0.0, 3.0, 0.0, 0.0, 0.0));

  EXPECT_GE(bb1.w, bb2.w);
  EXPECT_GE(bb1.h, bb2.h);
  EXPECT_LE(bb1.posX, bb2.posX);
  EXPECT_LE(bb1.posY, bb2.posY);
}

TEST( MathTest, pose2BBoxMoveOxSimulation)
{
  ros::param::set("/camera/d1", 0.0);
  ros::param::set("/camera/d2", 0.0);
  ros::param::set("/camera/d3", 0.0);
  ros::param::set("/camera/d4", 0.0);
  ros::param::set("/camera/d5", 0.0);

  Drone dr("dronych");

  Math* math = new Math();
  BBox bb1 = math->pose2BBox(dr, math->rotate2ClientSimulation(Pose(1.0, -0.1, 0.0, 0.0, 0.0, 0.0)));
  BBox bb2 = math->pose2BBox(dr, math->rotate2ClientSimulation(Pose(1.0, -0.9, 0.0, 0.0, 0.0, 0.0)));

  EXPECT_LE(bb1.posX, bb2.posX);
  EXPECT_NEAR(bb1.posY, bb2.posY, 0.01);
  EXPECT_NEAR(bb1.w, bb2.w, 0.01);
  EXPECT_NEAR(bb1.h, bb2.h, 0.01);
}

TEST( MathTest, pose2BBoxMoveOySimulation)
{
  ros::param::set("/camera/d1", 0.0);
  ros::param::set("/camera/d2", 0.0);
  ros::param::set("/camera/d3", 0.0);
  ros::param::set("/camera/d4", 0.0);
  ros::param::set("/camera/d5", 0.0);

  Drone dr("dronych");

  Math* math = new Math();
  BBox bb1 = math->pose2BBox(dr, math->rotate2ClientSimulation(Pose(1.0, 0.0, 0.5, 0.0, 0.0, 0.0)));
  BBox bb2 = math->pose2BBox(dr, math->rotate2ClientSimulation(Pose(1.0, 0.0, 0.1, 0.0, 0.0, 0.0)));

  EXPECT_LE(bb1.posY, bb2.posY);
  EXPECT_NEAR(bb1.posX, bb2.posX, 0.01);
  EXPECT_NEAR(bb1.w, bb2.w, 0.01);
  EXPECT_NEAR(bb1.h, bb2.h, 0.01);
}

TEST( MathTest, pose2BBoxMoveOzSimulation)
{
  ros::param::set("/camera/d1", 0.0);
  ros::param::set("/camera/d2", 0.0);
  ros::param::set("/camera/d3", 0.0);
  ros::param::set("/camera/d4", 0.0);
  ros::param::set("/camera/d5", 0.0);

  Drone dr("dronych");

  Math* math = new Math();
  BBox bb1 = math->pose2BBox(dr, math->rotate2ClientSimulation(Pose(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)));
  BBox bb2 = math->pose2BBox(dr, math->rotate2ClientSimulation(Pose(5.0, 0.0, 0.0, 0.0, 0.0, 0.0)));

  EXPECT_GE(bb1.w, bb2.w);
  EXPECT_GE(bb1.h, bb2.h);
  EXPECT_LE(bb1.posX, bb2.posX);
  EXPECT_LE(bb1.posY, bb2.posY);
}

TEST( MathTest, pose2BBoxMoveCameraOx)
{
  ros::param::set("/camera/d1", 0.0);
  ros::param::set("/camera/d2", 0.0);
  ros::param::set("/camera/d3", 0.0);
  ros::param::set("/camera/d4", 0.0);
  ros::param::set("/camera/d5", 0.0);

  ros::param::set("/camera/x", -0.5);
  ros::param::set("/camera/y", 0.0);
  ros::param::set("/camera/z", 0.0);

  Drone dr("dronych");

  Math* math = new Math();
  BBox bb1 = math->pose2BBox(dr, math->rotate2ClientSimulation(Pose(1.0, 0.0, 0.0, 0.0, 0.0, 0.0)));
  BBox bb2 = math->pose2BBox(dr, math->rotate2ClientSimulation(Pose(2.0, 0.0, 0.0, 0.0, 0.0, 0.0)));

  EXPECT_GE(bb1.w, bb2.w);
  EXPECT_GE(bb1.h, bb2.h);
  EXPECT_LE(bb1.posX, bb2.posX);
  EXPECT_LE(bb1.posY, bb2.posY);
}

TEST( MathTest, rotate2DroneNoAzimuthBasis)
{
  // --- Easy test, no rotation of the drone
  Pose current_pose(0.0, 0.0, 10.0, 0.0, 0.0, 0.0);

  // Rotate Ox
  Pose vector(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  Math* math = new Math();
  Pose result = math->rotate2Drone(current_pose, vector);

  EXPECT_EQ(0.0, result.posX);
  EXPECT_EQ(-1.0, result.posY);
  EXPECT_EQ(0.0, result.posZ);
  EXPECT_EQ(0.0, result.rotZ);

  // Rotate Oy
  vector.posX = 0.0;
  vector.posY = 1.0;
  vector.posZ = 0.0;

  result = math->rotate2Drone(current_pose, vector);

  EXPECT_EQ(0.0, result.posX);
  EXPECT_EQ(0.0, result.posY);
  EXPECT_EQ(-1.0, result.posZ);
  EXPECT_EQ(0.0, result.rotZ);

  // Rotate Oz
  vector.posX = 0.0;
  vector.posY = 0.0;
  vector.posZ = 1.0;

  result = math->rotate2Drone(current_pose, vector);

  EXPECT_EQ(1.0, result.posX);
  EXPECT_EQ(0.0, result.posY);
  EXPECT_EQ(0.0, result.posZ);
  EXPECT_EQ(0.0, result.rotZ);
}

TEST( MathTest, rotate2DroneNoAzimuthVector)
{
  // --- Easy test, no rotation of the drone
  Pose current_pose(0.0, 0.0, 10.0, 0.0, 0.0, 0.0);

  Pose vector(10.0, 10.0, 0.0, 0.0, 0.0, 0.0);

  Math* math = new Math();
  Pose result = math->rotate2Drone(current_pose, vector);

  EXPECT_EQ(0.0, result.posX);
  EXPECT_EQ(-10.0, result.posY);
  EXPECT_EQ(-10.0, result.posZ);
  EXPECT_EQ(0.0, result.rotZ);
}

TEST( MathTest, rotate2DroneNoAzimuthMoveOy)
{
  // --- Easy test, no rotation of the drone
  Pose current_pose(0.0, 0.0, 10.0, 0.0, 0.0, 0.0);

  Pose vector(10.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  Math* math = new Math();
  Pose result = math->rotate2Drone(current_pose, vector);

  EXPECT_EQ(0.0, result.posX);
  EXPECT_EQ(-10.0, result.posY);
  EXPECT_EQ(0.0, result.posZ);
  EXPECT_EQ(0.0, result.rotZ);

  vector.posX += 10.0;
  result = math->rotate2Drone(current_pose, vector);

  EXPECT_EQ(0.0, result.posX);
  EXPECT_EQ(-20.0, result.posY);
  EXPECT_EQ(0.0, result.posZ);
  EXPECT_EQ(0.0, result.rotZ);
}

TEST( MathTest, rotate2DroneWithRotationBasis)
{
  // --- Test with 180 rotation of the drones
  Pose current_pose(0.0, 0.0, 10.0, 0.0, 0.0, 180.0);

  Pose vector(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);

  Math* math = new Math();
  Pose result = math->rotate2Drone(current_pose, vector);

  EXPECT_NEAR(0.0, result.posX, std::pow(10,-10));
  EXPECT_NEAR(1.0, result.posY, std::pow(10,-10));
  EXPECT_NEAR(0.0, result.posZ, std::pow(10,-10));
  EXPECT_NEAR(0.0, result.rotZ, std::pow(10,-10));

  vector.posX = 0.0;
  vector.posY = 1.0;
  vector.posZ = 0.0;

  result = math->rotate2Drone(current_pose, vector);

  EXPECT_NEAR(0.0, result.posX, std::pow(10,-10));
  EXPECT_NEAR(0.0, result.posY, std::pow(10,-10));
  EXPECT_NEAR(-1.0, result.posZ, std::pow(10,-10));
  EXPECT_NEAR(0.0, result.rotZ, std::pow(10,-10));

  vector.posX = 0.0;
  vector.posY = 0.0;
  vector.posZ = 1.0;

  result = math->rotate2Drone(current_pose, vector);

  EXPECT_NEAR(-1.0, result.posX, std::pow(10,-10));
  EXPECT_NEAR(0.0, result.posY, std::pow(10,-10));
  EXPECT_NEAR(0.0, result.posZ, std::pow(10,-10));
  EXPECT_NEAR(0.0, result.rotZ, std::pow(10,-10));
}

TEST( MathTest, rotate2DroneWithRotationVector)
{
  // --- Test with 180 rotation of the drones
  Pose current_pose(0.0, 0.0, 10.0, 0.0, 0.0, 180.0);

  Pose vector(1.0, 1.0, 0.0, 0.0, 0.0, 0.0);

  Math* math = new Math();
  Pose result = math->rotate2Drone(current_pose, vector);

  EXPECT_NEAR(0.0, result.posX, std::pow(10,-10));
  EXPECT_NEAR(1.0, result.posY, std::pow(10,-10));
  EXPECT_NEAR(-1.0, result.posZ, std::pow(10,-10));
  EXPECT_NEAR(0.0, result.rotZ, std::pow(10,-10));
}

TEST( MathTest, distanceVectorFromSimulator)
{
  Drone dr("parrot_bebop2");
  BBox current(16.8514, 42.9556, 66.0678, 20.5102, 0.0, 0.0, 0.0);
  dr.setCurrent(current);

  Math* math = new Math();
  Pose result = math->distanceVector(dr, Drone::BBoxKind::CURRENT);

  EXPECT_NEAR(-0.0001, result.posX, 0.001);
  EXPECT_NEAR(0.0151, result.posY, 0.001);
  EXPECT_NEAR(0.4999, result.posZ, 0.001);
}

TEST( MathTest, iouFullOverlap)
{
  BBox from(0.0, 0.0, 20.0, 20.0, 0.0, 0.0, 0.0);
  BBox to(0.0, 0.0, 20.0, 20.0, 0.0, 0.0, 0.0);

  Math* math = new Math();
  EXPECT_EQ(1.0, math->iou(from, to));
}

TEST( MathTest, iouNoOverlap)
{
  BBox from(0.0, 0.0, 20.0, 20.0, 0.0, 0.0, 0.0);
  BBox to(50.0, 50.0, 20.0, 20.0, 0.0, 0.0, 0.0);

  Math* math = new Math();
  EXPECT_EQ(0.0, math->iou(from, to));
}

TEST( MathTest, iouStrongOverlap)
{
  BBox from(0.0, 0.0, 20.0, 20.0, 0.0, 0.0, 0.0);
  BBox to(1.0, 1.0, 20.0, 20.0, 0.0, 0.0, 0.0);

  Math* math = new Math();
  double result = math->iou(from, to);
  EXPECT_TRUE((result < 1) && (result > 0.5));
}

TEST( MathTest, iouWeakOverlap)
{
  BBox from(0.0, 0.0, 20.0, 20.0, 0.0, 0.0, 0.0);
  BBox to(19.0, 19.0, 20.0, 20.0, 0.0, 0.0, 0.0);

  Math* math = new Math();
  double result = math->iou(from, to);
  EXPECT_TRUE((result < 0.5) && (result > 0.0));
}

TEST( MathTest, iouNestedOverlap)
{
  BBox from(0.0, 0.0, 20.0, 20.0, 0.0, 0.0, 0.0);
  BBox to(0.0, 0.0, 19.0, 19.0, 0.0, 0.0, 0.0);

  Math* math = new Math();
  double result = math->iou(from, to);

  EXPECT_TRUE((result < 1) && (result > 0.5));
}

TEST( MathTest, distanceFromSimulation)
{
  Drone dr("parrot_bebop2");

  BBox current(16.8514, 42.9556, 66.0678, 20.5102, 0.0, 0.0, 0.0);
  BBox target(38.8924, 47.4715, 21.9929, 6.8275, 0.0, 0.0, 0.0);

  dr.setCurrent(current);
  dr.setTarget(target);

  Math* math = new Math();
  double distance = math->distance(
                    dr,
                    Drone::BBoxKind::CURRENT,
                    Drone::BBoxKind::TARGET);
  EXPECT_NEAR( 1.0, distance, 0.002);

  BBox target2(10.1067, 47.4715, 21.9929, 6.8275, 0.0, 0.0, 0.0);

  dr.setCurrent(target);
  dr.setTarget(target2);

  distance = math->distance(
                    dr,
                    Drone::BBoxKind::CURRENT,
                    Drone::BBoxKind::TARGET);

  EXPECT_NEAR( 0.5, distance, 0.002);
}

TEST( MathTest, transform)
{
  Drone dr("parrot_bebop2");

  BBox current(16., 42., 66.0678, 20.5102, 0.0, 0.0, 0.0);
  BBox target(16., 42., 21.9929, 6.8275, 0.0, 0.0, 0.0);

  dr.setCurrent(current);
  dr.setTarget(target);

  Math* math = new Math();
  Pose result = math->transform(dr);

  EXPECT_NE(0.0, result.posZ);
}


int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);

    ros::init(argc, argv, "RegulatorUtilTest");

    ros::param::set("/camera/fx", 5.5322048754249238e+02);
    ros::param::set("/camera/fy", 5.5285611970941648e+02);
    ros::param::set("/camera/cx", 3.1929420090888715e+02);
    ros::param::set("/camera/cy", 2.3867960696716827e+02);

    ros::param::set("/camera/d1", 0.0);
    ros::param::set("/camera/d2", 0.0);
    ros::param::set("/camera/d3", 0.0);
    ros::param::set("/camera/d4", 0.0);
    ros::param::set("/camera/d5", 0.0);

    ros::param::set("/camera/x", -0.5);
    ros::param::set("/camera/y", 0.0);
    ros::param::set("/camera/z", 0.0);

    ros::param::set("/drones/dronych/size/width", 0.2);
    ros::param::set("/drones/dronych/size/height", 0.1);

    ros::param::set("/drones/parrot_bebop2/size/width", 0.382);
    ros::param::set("/drones/parrot_bebop2/size/height", 0.089);
    ros::param::set("/drones/parrot_bebop2/size/depth", 0.328);

    return RUN_ALL_TESTS();
}
