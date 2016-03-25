#if 0
// Core ros functionality like ros::init and spin
#include <ros/ros.h>
// ROS Trajectory Action server definition
#include <control_msgs/FollowJointTrajectoryAction.h>
// Means by which we communicate with above action-server
#include <actionlib/client/simple_action_client.h>

// Includes the descartes robot model we will be using
#include <descartes_moveit/moveit_state_adapter.h>
// Includes the descartes trajectory type we will be using
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/cart_trajectory_pt.h>
// Includes the planner we will be using
#include <descartes_planner/dense_planner.h>
#include <descartes_planner/sparse_planner.h>

// Includes MoveIt
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/move_group_interface/move_group.h>


typedef std::vector<descartes_core::TrajectoryPtPtr> DescartesTrajectory;
typedef DescartesTrajectory::const_iterator TrajectoryIter;

#endif

#include "ra605_descartes/path_gen.h"

namespace ra605_descartes
{


#if 0
bool IsPerpendicular(Eigen::Vector3d *pt1, Eigen::Vector3d *pt2, Eigen::Vector3d *pt3);
double CalcCircleCenter(Eigen::Vector3d *pt1, Eigen::Vector3d *pt2, Eigen::Vector3d *pt3, Eigen::Vector3d *center);

void FindCircleCenter(const Eigen::Vector3d *V1, const Eigen::Vector3d *V2, const Eigen::Vector3d *V3, Eigen::Vector3d *center)
{
    Eigen::Vector3d *pt1=new Eigen::Vector3d(*V1);
    Eigen::Vector3d *pt2=new Eigen::Vector3d(*V2);
    Eigen::Vector3d *pt3=new Eigen::Vector3d(*V3);


    if (!IsPerpendicular(pt1, pt2, pt3) )       CalcCircleCenter(pt1, pt2, pt3, center);
    else if (!IsPerpendicular(pt1, pt3, pt2) )  CalcCircleCenter(pt1, pt3, pt2, center);
    else if (!IsPerpendicular(pt2, pt1, pt3) )  CalcCircleCenter(pt2, pt1, pt3, center);
    else if (!IsPerpendicular(pt2, pt3, pt1) )  CalcCircleCenter(pt2, pt3, pt1, center);
    else if (!IsPerpendicular(pt3, pt2, pt1) )  CalcCircleCenter(pt3, pt2, pt1, center);
    else if (!IsPerpendicular(pt3, pt1, pt2) )  CalcCircleCenter(pt3, pt1, pt2, center);
    else {
        delete pt1;
        delete pt2;
        delete pt3;
        return;
    }
    delete pt1;
    delete pt2;
    delete pt3;

}




#endif

descartes_core::TrajectoryPtPtr PathGen::makeCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose)) );
}

// TODO: set tolerance and axis as parameters
descartes_core::TrajectoryPtPtr PathGen::makeTolerancedCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/4.0, AxialSymmetricPt::Z_AXIS) );
}

bool PathGen::IsPerpendicular(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2, const Eigen::Vector3d& pt3)
// Check the given point are perpendicular to x or y axis
{
    double yDelta_a= pt2(1) - pt1(1); // y
    double xDelta_a= pt2(0) - pt1(0); // x
    double yDelta_b= pt3(1) - pt2(1); // y
    double xDelta_b= pt3(0) - pt2(0); // x

    // checking whether the line of the two pts are vertical
    if (fabs(xDelta_a) <= 0.000000001 && fabs(yDelta_b) <= 0.000000001){
        return false;
    }

    if (fabs(yDelta_a) <= 0.0000001){
        return true;
    }
    else if (fabs(yDelta_b) <= 0.0000001){
        return true;
    }
    else if (fabs(xDelta_a)<= 0.000000001){
        return true;
    }
    else if (fabs(xDelta_b)<= 0.000000001){
        return true;
    }
    else
        return false ;
}

int PathGen::CalcCircleCenter(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2, const Eigen::Vector3d& pt3, Eigen::Vector3d& center)
{
    double yDelta_a = pt2(1) - pt1(1); // y
    double xDelta_a = pt2(0) - pt1(0); // x
    double yDelta_b = pt3(1) - pt2(1); // y
    double xDelta_b = pt3(0) - pt2(0); // x

    if (fabs(xDelta_a) <= 0.000000001 && fabs(yDelta_b) <= 0.000000001){
        center(0) = 0.5*(pt2(0) + pt3(0)); // x
        center(1) = 0.5*(pt1(1) + pt2(1)); // y
        center(2) = pt1(2); // z
        return 0;
    }

    // IsPerpendicular() assure that xDelta(s) are not zero
    double aSlope=yDelta_a/xDelta_a; //
    double bSlope=yDelta_b/xDelta_b;
    if (fabs(aSlope-bSlope) <= 0.000000001){    // checking whether the given points are colinear.
        ROS_ERROR("(%s:%d) Cannot find center, colinear points!", __FILE__, __LINE__);
        return -1;
    }

    // calc center
    center(0) = (aSlope*bSlope*(pt1(1) - pt3(1)) + bSlope*(pt1(0) + pt2(0))
                         - aSlope*(pt2(0)+pt3(0)) )/(2* (bSlope-aSlope) ); // x
    center(1) = -1*(center(0) - (pt1(0)+pt2(0))/2)/aSlope +  (pt1(1)+pt2(1))/2; // y
    center(2) = pt1(2); // z

    return 0;
}

void PathGen::FindCircleCenter(const Eigen::Vector3d& pt1, const Eigen::Vector3d& pt2, const Eigen::Vector3d& pt3, Eigen::Vector3d& center)
{
    if      (!IsPerpendicular(pt1, pt2, pt3) )  CalcCircleCenter(pt1, pt2, pt3, center);
    else if (!IsPerpendicular(pt1, pt3, pt2) )  CalcCircleCenter(pt1, pt3, pt2, center);
    else if (!IsPerpendicular(pt2, pt1, pt3) )  CalcCircleCenter(pt2, pt1, pt3, center);
    else if (!IsPerpendicular(pt2, pt3, pt1) )  CalcCircleCenter(pt2, pt3, pt1, center);
    else if (!IsPerpendicular(pt3, pt2, pt1) )  CalcCircleCenter(pt3, pt2, pt1, center);
    else if (!IsPerpendicular(pt3, pt1, pt2) )  CalcCircleCenter(pt3, pt1, pt2, center);
    else {
        ROS_ERROR("(%s:%d) Cannot find center!", __FILE__, __LINE__);
        return;
    }
}

// http://stackoverflow.com/questions/13977354/build-circle-from-3-points-in-3d-space-implementation-in-c-or-c
void PathGen::buildCircleBy3Pt(
        const Eigen::Vector3d& p1_i,
        const Eigen::Vector3d& p2_i,
        const Eigen::Vector3d& p3_i,
        Eigen::Vector3d& center,
        DescartesTrajectory& points,
        double resolution)
{
    /*  Get the normal vector to the triangle formed by 3 points
        Calc a rotation quaternion from that normal to the 0,0,1 axis
        Rotate 3 points using quaternion. Points will be in XY plane
        Build a circle by 3 points on XY plane
        Rotate a circle back into original plane using quaternion
     */
    const Eigen::Vector3d p2top1 = p1_i - p2_i;
    const Eigen::Vector3d p2top3 = p3_i - p2_i;

    const Eigen::Vector3d circle_normal((p2top1.cross(p2top3)).normalized());
    const Eigen::Vector3d xy_normal(0, 0, 1);

    Eigen::Matrix3d rot_quat;
    rot_quat = Eigen::Quaterniond().setFromTwoVectors(xy_normal,circle_normal);

    Eigen::Matrix3d rot_back_quat;
    rot_back_quat = Eigen::Quaterniond().setFromTwoVectors(circle_normal, xy_normal);

    Eigen::Vector3d p1 = rot_quat * p1_i;
    Eigen::Vector3d p2 = rot_quat * p2_i;
    Eigen::Vector3d p3 = rot_quat * p3_i;

    ROS_INFO_STREAM("p2top1: ["
                    <<p2top1(0)<<", "
                    <<p2top1(1)<<", "
                    <<p2top1(2)<<"] ");

    ROS_INFO_STREAM("circle_normal: ["
                    <<circle_normal(0)<<", "
                    <<circle_normal(1)<<", "
                    <<circle_normal(2)<<"] ");

    ROS_INFO_STREAM("p1: ["
                    <<p1(0)<<", "
                    <<p1(1)<<", "
                    <<p1(2)<<"] ");

    ROS_INFO_STREAM("p2: ["
                    <<p2(0)<<", "
                    <<p2(1)<<", "
                    <<p2(2)<<"] ");

    ROS_INFO_STREAM("p3: ["
                    <<p3(0)<<", "
                    <<p3(1)<<", "
                    <<p3(2)<<"] ");

    ROS_INFO_STREAM("center: ["
                    <<center(0)<<", "
                    <<center(1)<<", "
                    <<center(2)<<"] ");

    // calculate 2D center
    FindCircleCenter(p1, p2, p3, center);

    // calc radius
    const double radius = (center - p1).norm();

    // calc angle
    const Eigen::Vector3d rTan = p1 - center;
    const Eigen::Vector3d rEnd = p3 - center;
    double dot = rTan.dot(rEnd) / (radius * radius);
    double angle;
    if (dot > 1.0) {
        angle = 0.0;
    } else if (dot < -1.0) {
        angle = M_PI;
    } else {
        angle = std::acos(dot);
    }
    ROS_INFO_STREAM("dot: [" <<dot<<"] ");
    ROS_INFO_STREAM("angle: [" <<angle<<"] ");

    /* now angle is in range 0..PI . Check if cross is antiparallel to
       normal. If so, true angle is between PI..2PI. Need to subtract from
       2PI. */
    const Eigen::Vector3d circle_cross = rTan.cross(rEnd);
    dot = circle_cross.dot(xy_normal);
    if (dot < 0.0) {
        angle = 2 * M_PI - angle;
    }

    double arc_length = radius * angle;

    ROS_INFO_STREAM("radius: [" <<radius<<"] ");
    ROS_INFO_STREAM("angle1: [" <<angle<<"] ");
    ROS_INFO_STREAM("dot1: [" <<dot<<"] ");
    ROS_INFO_STREAM("arc_length: [" <<arc_length<<"] ");
    ROS_INFO_STREAM("rTan"<<": ["<<rTan(0)<<", "<<rTan(1)<<"] ");
    ROS_INFO_STREAM("rEnd"<<": ["<<rEnd(0)<<", "<<rEnd(1)<<"] ");

    const double arc_coef = resolution / arc_length;
    double cur_angle;
    int i;
    for (i=0; i < (arc_length / resolution); i++) {
        cur_angle = angle * i * arc_coef;
        Eigen::Rotation2Dd r(cur_angle);
        Eigen::Vector2d rTan2d(rTan(0), rTan(1));
        Eigen::Vector2d pt2d = r * rTan2d;      // rotate start-point(rTan) with cur_angle
        ROS_INFO_STREAM("pt2d"<<i<<": ["<<pt2d(0)<<", "<<pt2d(1)<<"] ");
        // rotate the point back into original plane
        Eigen::Vector3d pt = rot_back_quat * Eigen::Vector3d(pt2d(0) + center(0), pt2d(1) + center(1), center(2));
        ROS_INFO_STREAM("pt3d"<<i<<": ["<<pt(0)<<", "<<pt(1)<<", "<<pt(2)<<"] ");
        Eigen::Affine3d pose = descartes_core::utils::toFrame(  pt(0),              //x
                                                                pt(1),              //y
                                                                pt(2),              //z
                                                                0,                 //rx
                                                                M_PI,              //ry
                                                                0);                 //rz
        descartes_core::TrajectoryPtPtr tpt = makeTolerancedCartesianPoint(pose);
        points.push_back(tpt);
    }
    center = rot_back_quat * center; // update arc center
}

} // namespace ra605_descartes
