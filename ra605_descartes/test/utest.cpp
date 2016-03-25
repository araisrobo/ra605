/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <gtest/gtest.h>
#include <climits>
#include <ra605_descartes/path_gen.h>

using namespace ra605_descartes;

TEST(PathGenTest, IsPerpendicularFunc){
    PathGen pg;

    double x_base = 0.138;
    double y_base = 0.15;
    double z_base = -0.35;

    // 1. Define 3 points for an arc on xy-plain
    Eigen::Vector3d pt1(x_base,         y_base,         z_base);
    Eigen::Vector3d pt2(x_base + 0.006, y_base + 0.002, z_base);
    Eigen::Vector3d pt3(x_base + 0.01,  y_base + 0.01,  z_base);

    ASSERT_EQ (false, pg.IsPerpendicular(pt1, pt2, pt3));
    ASSERT_EQ (false, pg.IsPerpendicular(pt1, pt3, pt2));
    ASSERT_EQ (false, pg.IsPerpendicular(pt2, pt1, pt3));
    ASSERT_EQ (false, pg.IsPerpendicular(pt2, pt3, pt1));
    ASSERT_EQ (false, pg.IsPerpendicular(pt3, pt2, pt1));
    ASSERT_EQ (false, pg.IsPerpendicular(pt3, pt1, pt2));
}

TEST(PathGenTest, CalcCircleCenterFunc){
    PathGen pg;

    double x_base = 0.138;
    double y_base = 0.15;
    double z_base = -0.35;

    // 1. Define 3 points for an arc on xy-plain
    Eigen::Vector3d pt1(x_base,         y_base,         z_base);
    Eigen::Vector3d pt2(x_base + 0.006, y_base + 0.002, z_base);
    Eigen::Vector3d pt3(x_base + 0.01,  y_base + 0.01,  z_base);
    Eigen::Vector3d center;
    ROS_DEBUG("pt1(%g, %g, %g)", pt1(0), pt1(1), pt1(2));
    ROS_DEBUG("pt2(%g, %g, %g)", pt2(0), pt2(1), pt2(2));
    ROS_DEBUG("pt3(%g, %g, %g)", pt3(0), pt3(1), pt3(2));

    ASSERT_EQ (0, pg.CalcCircleCenter(pt1, pt2, pt3, center));
    ROS_DEBUG("center(%g, %g, %g)", center(0), center(1), center(2));

    ASSERT_EQ (0, pg.CalcCircleCenter(pt1, pt3, pt2, center));
    ROS_DEBUG("center(%g, %g, %g)", center(0), center(1), center(2));

    ASSERT_EQ (0, pg.CalcCircleCenter(pt2, pt1, pt3, center));
    ASSERT_EQ (0, pg.CalcCircleCenter(pt2, pt3, pt1, center));
    ASSERT_EQ (0, pg.CalcCircleCenter(pt3, pt2, pt1, center));
    ASSERT_EQ (0, pg.CalcCircleCenter(pt3, pt1, pt2, center));
}

TEST(PathGenTest, FindCircleCenterFunc){
    PathGen pg;

    double x_base = 0.138;
    double y_base = 0.15;
    double z_base = -0.35;

    // 1. Define 3 points for an arc on xy-plain
    Eigen::Vector3d pt1(x_base,         y_base,         z_base);
    Eigen::Vector3d pt2(x_base + 0.006, y_base + 0.002, z_base);
    Eigen::Vector3d pt3(x_base + 0.01,  y_base + 0.01,  z_base);
    Eigen::Vector3d center;
    ROS_DEBUG("pt1(%g, %g, %g)", pt1(0), pt1(1), pt1(2));
    ROS_DEBUG("pt2(%g, %g, %g)", pt2(0), pt2(1), pt2(2));
    ROS_DEBUG("pt3(%g, %g, %g)", pt3(0), pt3(1), pt3(2));

    pg.FindCircleCenter(pt1, pt2, pt3, center);
    ASSERT_EQ (0.138, center(0));
    ASSERT_EQ (0.16,  center(1));
    ASSERT_EQ (-0.35, center(2));
    ROS_DEBUG("center(%g, %g, %g)", center(0), center(1), center(2));
}

TEST(PathGenTest, buildCircleBy3PtFunc){
    PathGen pg;
    DescartesTrajectory points;

    double x_base = 0.35; // was 0.05
    double y_base = 0.15;  // was 0.5
    double z_base = 0.123; // was 0.35

    // 1. Define 3 points for an arc on xy-plain
    Eigen::Vector3d pt1(x_base, y_base, z_base + 0.015);
    Eigen::Vector3d pt2(x_base, y_base + 0.002, z_base + 0.015 + 0.006);
    Eigen::Vector3d pt3(x_base, y_base + 0.01, z_base + 0.025);
    Eigen::Vector3d center;

    points.clear();
    pg.buildCircleBy3Pt(pt1, pt2, pt3, center, points, 0.001);

    ROS_DEBUG("pt1(%g, %g, %g)", pt1(0), pt1(1), pt1(2));
    ROS_DEBUG("pt2(%g, %g, %g)", pt2(0), pt2(1), pt2(2));
    ROS_DEBUG("pt3(%g, %g, %g)", pt3(0), pt3(1), pt3(2));
    ROS_DEBUG("center(%g, %g, %g)", center(0), center(1), center(2));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
//  ros::init(argc, argv, "utest");
//  boost::thread ros_thread(boost::bind(&ros::spin));

  int res = RUN_ALL_TESTS();
//  ros_thread.interrupt();
//  ros_thread.join();

  return res;
}
