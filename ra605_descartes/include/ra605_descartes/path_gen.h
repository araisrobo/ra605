/*
 * ra605_robot_model.h
 *
 *  Created on: Apr 15, 2015
 *      Author: ros-devel
 */

#ifndef RA605_DESCARTES_PATH_GEN_H_
#define RA605_DESCARTES_PATH_GEN_H_

//#include <descartes_moveit/moveit_state_adapter.h>
//#include <ros/ros.h>
//#include <urdf/model.h>
//#include <eigen_conversions/eigen_kdl.h>
//#include <tf_conversions/tf_kdl.h>

// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>

namespace ra605_descartes
{

typedef std::vector<descartes_core::TrajectoryPtPtr> DescartesTrajectory;

class PathGen
{
public:
  PathGen(){};
//  virtual ~PathGen();

  /**
   * Generates an completely defined (zero-tolerance) cartesian point from a pose
   */
  descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose);
  /**
   * Generates a cartesian point with free rotation about the Z axis of the EFF frame
   */
  descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose);

  bool IsPerpendicular(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2, const Eigen::Vector3d& pt3);
  int CalcCircleCenter(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2, const Eigen::Vector3d& pt3, Eigen::Vector3d& center);
  void FindCircleCenter(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2, const Eigen::Vector3d& pt3, Eigen::Vector3d& center);

  void buildCircleBy3Pt(
          const Eigen::Vector3d& p1_i,
          const Eigen::Vector3d& p2_i,
          const Eigen::Vector3d& p3_i,
          Eigen::Vector3d& center,
          DescartesTrajectory& points,
          double resolution);
};

} /* namespace ra605_descartes */

#endif /* RA605_DESCARTES_PATH_GEN_H_ */
