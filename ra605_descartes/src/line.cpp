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

// Includes MoveIt
#include <moveit_msgs/ExecuteKnownTrajectory.h>
#include <moveit/move_group_interface/move_group.h>


typedef std::vector<descartes_core::TrajectoryPtPtr> DescartesTrajectory;
typedef DescartesTrajectory::const_iterator TrajectoryIter;

/*  =============================== Application Data Structure ===============================
 *
 * Holds the data used at various points in the application.  This structure is populated
 * from data found in the ros parameter server at runtime.
 *
 */
struct DemoConfiguration
{
  std::string group_name;                 /* Name of the manipulation group containing the relevant links in the robot */
  std::string tip_link;                   /* Usually the last link in the kinematic chain of the robot */
  std::string base_link;                  /* The name of the base link of the robot */
  std::string world_frame;                /* The name of the world link in the URDF file */
  std::vector<std::string> joint_names;   /* A list with the names of the mobile joints in the robot */


  /* Trajectory Generation Members:
   *  Used to control the attributes (points, shape, size, etc) of the robot trajectory.
   *  */
  double time_delay;              /* Time step between consecutive points in the robot path */
  double delta_dist;              /* delta distance between two consecutive points in the robot path */
  double foci_distance;           /* Controls the size of the curve */
  double radius;                  /* Controls the radius of the sphere on which the curve is projected */
  int num_points;                 /* Number of points per curve */
  int num_lemniscates;            /* Number of curves*/
  std::vector<double> center;     /* Location of the center of all the lemniscate curves */
  std::vector<double> seed_pose;  /* Joint values close to the desired start of the robot path */

  /*
   * Visualization Members
   * Used to control the attributes of the visualization artifacts
   */
  double min_point_distance;      /* Minimum distance between consecutive trajectory points. */
};

/* Application Data
 *  Holds the data used by the various functions in the application.
 */
DemoConfiguration config_;

//ros::NodeHandle nh_;                      /* Object used for creating and managing ros application resources*/
ros::Publisher marker_publisher_;           /* Publishes visualization message to Rviz */
ros::ServiceClient moveit_run_path_client_; /* Sends a robot trajectory to moveit for execution */

/**
 * Generates an completely defined (zero-tolerance) cartesian point from a pose
 */
descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose);
/**
 * Generates a cartesian point with free rotation about the Z axis of the EFF frame
 */
descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose);

/**
 * Translates a descartes trajectory to a ROS joint trajectory
 */
trajectory_msgs::JointTrajectory
toROSJointTrajectory(const DescartesTrajectory& trajectory, const descartes_core::RobotModel& model,
                     const std::vector<std::string>& joint_names, double time_delay);

void runPath(const DescartesTrajectory& path,
             const descartes_core::RobotModelPtr robot_model_ptr_);

/**
 * Sends a ROS trajectory to the robot controller
 */
bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);

const std::string VISUALIZE_TRAJECTORY_TOPIC = "visualize_trajectory_curve";
const std::string EXECUTE_TRAJECTORY_SERVICE = "execute_kinematic_path";
const double SERVICE_TIMEOUT = 5.0f; // seconds

//int createLine(double foci_distance, double sphere_radius,
//                                  int num_points, int num_lemniscates,const Eigen::Vector3d& sphere_center,
//                                  EigenSTL::vector_Affine3d& poses)
//{
//  double a = foci_distance;
//  double ro = sphere_radius;
//  int npoints = num_points;
//  int nlemns = num_lemniscates;
//  Eigen::Vector3d offset(sphere_center[0],sphere_center[1],sphere_center[2]);
//  Eigen::Vector3d unit_z,unit_y,unit_x;
//
//  // checking parameters
//  if(a <= 0 || ro <= 0 || npoints < 10 || nlemns < 1)
//  {
//    ROS_ERROR_STREAM("Invalid parameters for lemniscate curve were found");
//    return -1;
//  }
//
//  // generating polar angle values
//  std::vector<double> theta(npoints);
//
//  // interval 1 <-pi/4 , pi/4 >
//  double d_theta = 2*M_PI_2/(npoints - 1);
//  for(unsigned int i = 0; i < npoints/2;i++)
//  {
//    theta[i] = -M_PI_4  + i * d_theta;
//  }
//  theta[0] = theta[0] + EPSILON;
//  theta[npoints/2 - 1] = theta[npoints/2 - 1] - EPSILON;
//
//  // interval 2 < 3*pi/4 , 5 * pi/4 >
//  for(unsigned int i = 0; i < npoints/2;i++)
//  {
//    theta[npoints/2 + i] = 3*M_PI_4  + i * d_theta;
//  }
//  theta[npoints/2] = theta[npoints/2] + EPSILON;
//  theta[npoints - 1] = theta[npoints - 1] - EPSILON;
//
//  // generating omega angle (lemniscate angle offset)
//  std::vector<double> omega(nlemns);
//  double d_omega = M_PI/(nlemns);
//  for(unsigned int i = 0; i < nlemns;i++)
//  {
//     omega[i] = i*d_omega;
//  }
//
//  Eigen::Affine3d pose;
//  double x,y,z,r,phi;
//
//  poses.clear();
//  poses.reserve(nlemns*npoints);
//  for(unsigned int j = 0; j < nlemns;j++)
//  {
//    for(unsigned int i = 0 ; i < npoints;i++)
//    {
//      r = std::sqrt( std::pow(a,2) * std::cos(2*theta[i]) );
//      phi = r < ro ? std::asin(r/ro):  (M_PI - std::asin((2*ro - r)/ro) );
//
//      x = ro * std::cos(theta[i] + omega[j]) * std::sin(phi);
//      y = ro * std::sin(theta[i] + omega[j]) * std::sin(phi);
//      z = ro * std::cos(phi);
//
//      // determining orientation
//      unit_z <<-x, -y , -z;
//      unit_z.normalize();
//
//      unit_x = (Eigen::Vector3d(0,1,0).cross( unit_z)).normalized();
//      unit_y = (unit_z .cross(unit_x)).normalized();
//
//      Eigen::Matrix3d rot;
//      rot << unit_x(0),unit_y(0),unit_z(0)
//         ,unit_x(1),unit_y(1),unit_z(1)
//         ,unit_x(2),unit_y(2),unit_z(2);
//
//      pose = Eigen::Translation3d(offset(0) + x,
//                                  offset(1) + y,
//                                  offset(2) + z) * rot;
//
//      poses.push_back(pose);
//    }
//  }
//
//  return 0;
//}

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

bool IsPerpendicular(Eigen::Vector3d *pt1, Eigen::Vector3d *pt2, Eigen::Vector3d *pt3)
// Check the given point are perpendicular to x or y axis
{
    double yDelta_a= pt2->y - pt1->y;
    double xDelta_a= pt2->x - pt1->x;
    double yDelta_b= pt3->y - pt2->y;
    double xDelta_b= pt3->x - pt2->x;

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

double CalcCircleCenter(Eigen::Vector3d *pt1, Eigen::Vector3d *pt2, Eigen::Vector3d *pt3, Eigen::Vector3d *center)
{
    double yDelta_a = pt2->y - pt1->y;
    double xDelta_a = pt2->x - pt1->x;
    double yDelta_b = pt3->y - pt2->y;
    double xDelta_b = pt3->x - pt2->x;

    if (fabs(xDelta_a) <= 0.000000001 && fabs(yDelta_b) <= 0.000000001){
        center->x= 0.5*(pt2->x + pt3->x);
        center->y= 0.5*(pt1->y + pt2->y);
        center->z= pt1->z;

        return 1;
    }

    // IsPerpendicular() assure that xDelta(s) are not zero
    double aSlope=yDelta_a/xDelta_a; //
    double bSlope=yDelta_b/xDelta_b;
    if (fabs(aSlope-bSlope) <= 0.000000001){    // checking whether the given points are colinear.
        return -1;
    }

    // calc center
    center->x= (aSlope*bSlope*(pt1->y - pt3->y) + bSlope*(pt1->x + pt2 ->x)
                         - aSlope*(pt2->x+pt3->x) )/(2* (bSlope-aSlope) );
    center->y = -1*(center->x - (pt1->x+pt2->x)/2)/aSlope +  (pt1->y+pt2->y)/2;

    return 1;
}
#endif

// http://stackoverflow.com/questions/13977354/build-circle-from-3-points-in-3d-space-implementation-in-c-or-c



//Eigen::Vector3d pt1(0.05, -0.5, 0.365);
//Eigen::Vector3d pt2(0.05, -0.5, 0.365);
//Eigen::Vector3d pt3(0.06, -0.5, 0.375);
//Eigen::Vector3d center(0.06, -0.5, 0.365);
//buildCircleBy3Pt(&pt1, &pt2, &pt3, &center, &points);

void buildCircleBy3Pt(
        const Eigen::Vector3d& p1_i,
        const Eigen::Vector3d& p2_i,
        const Eigen::Vector3d& p3_i,
        const Eigen::Vector3d& center_i,
        DescartesTrajectory& points)
{
    /*  Get the normal vector to the triangle formed by 3 points
        Calc a rotation quaternion from that normal to the 0,0,1 axis
        Rotate 3 points using quaternion. Points will be in XY plane
        Build a circle by 3 points on XY plane
        Rotate a circle back into original plane using quaternion
     */
//    Eigen::Vector3d p1(1, 0, -1);
//    Eigen::Vector3d p2(1, 0, 1);
//    Eigen::Vector3d p3(2, 0, 0);
//    Eigen::Vector3d center(1, 0, 0);
//    if (c)
//    {
//        center.set(c[0], c[1], c[2]);
//    }

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
    Eigen::Vector3d center = rot_quat * center_i;

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
//    if (!c)
//    {
//        // calculate 2D center
//        FindCircleCenter(&p1, &p2, &p3, &center);
//    }

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

    const double arc_coef = 0.001 / arc_length;
    double cur_angle;
    int i;
    for (i=0; i < (arc_length * 1000); i++) {
        cur_angle = angle * i * arc_coef;
        Eigen::Rotation2Dd r(cur_angle);
        Eigen::Vector2d rTan2d(rTan(0), rTan(1));
        Eigen::Vector2d pt2d = r * rTan2d;      // rotate start-point(rTan) with cur_angle
        ROS_INFO_STREAM("pt2d"<<i<<": ["<<pt2d(0)<<", "<<pt2d(1)<<"] ");
        // rotate the point back into original plane
        Eigen::Vector3d pt = rot_back_quat * Eigen::Vector3d(pt2d(0) + center(0), pt2d(1) + center(1), + center(2));
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
}


int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "descartes_tutorial");
  ros::NodeHandle nh;

  // Required for communication with moveit components
  ros::AsyncSpinner spinner (1);
  spinner.start();

  //TODO: figure out how to setup nodeParameters, so that getParam will work for group_name
  //TODO: move to init_ros()
  config_.group_name = "arm1";
  config_.tip_link = "link6";
  config_.world_frame = "base_link";
  config_.delta_dist = 0.0001;
  nh.getParam("controller_joint_names",config_.joint_names);
  ROS_INFO_STREAM("group_name: " << config_.group_name);
  ROS_INFO_STREAM("tip_link: " << config_.tip_link);
  ROS_INFO_STREAM("world_frame: " << config_.world_frame);
  ROS_INFO_STREAM("joint_names[0]: " << config_.joint_names[0]);

  // creating publisher for trajectory visualization
  marker_publisher_  = nh.advertise<visualization_msgs::MarkerArray>(VISUALIZE_TRAJECTORY_TOPIC,1,true);

  /*  Fill Code:
   * Goal:
   * - Create a "moveit_msgs::ExecuteKnownTrajectory" client and assign it to the "moveit_run_path_client_"
   *    application variable.
   * Hint:
   * - Enter the service type moveit_msgs::ExecuteKnownTrajectory in between the "< >" arrow brackets of
   *   the "nh_.serviceClient" function call.
   */
  moveit_run_path_client_ = nh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(EXECUTE_TRAJECTORY_SERVICE,true);

  // Establishing connection to server
  if(moveit_run_path_client_.waitForExistence(ros::Duration(SERVICE_TIMEOUT)))
  {
    ROS_INFO_STREAM("Connected to '"<<moveit_run_path_client_.getService()<<"' service");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to connect to '"<< moveit_run_path_client_.getService()<<"' service");
    exit(-1);
  }

  // generating trajectory using a line function.
  EigenSTL::vector_Affine3d poses;
  Eigen::Affine3d begin_pose = descartes_core::utils::toFrame(
                                                          0.05f,             //x
                                                         -0.5f,              //y
                                                          0.35f,             //z
                                                          0,                 //rx
                                                          M_PI,              //ry
                                                          0);                //rz
//  if(createLine(config_.delta_dist, begin, end, poses))
//  {
//    ROS_INFO_STREAM("Trajectory with "<<poses.size()<<" points was generated");
//  }
//  else
//  {
//    ROS_ERROR_STREAM("Trajectory generation failed");
//    exit(-1);
//  }

//  // publishing trajectory poses for visualization
//  publishPosesMarkers(poses);

//  // creating descartes trajectory points
//  DescartesTrajectory traj;
//  traj.clear();
//  traj.reserve(poses.size());
//  for(unsigned int i = 0; i < poses.size(); i++)
//  {
//    const Eigen::Affine3d& pose = poses[i];
//
//    /*  Fill Code:
//     * Goal:
//     * - Create AxialSymetricPt objects in order to define a trajectory cartesian point with
//     *    rotational freedom about the tool's z axis.
//     *
//     * Hint:
//     * - The point can be constructed as follows:
//     *
//     *    new AxialSymmetricPt(Pose ,Increment, Free Axis)
//     *
//     * - The Pose can be found in the for loop's "pose" variable.
//     * - The Increment can be found in the "ORIENTATION_INCREMENT" global variable.
//     * - The Free Axis can be selected from the AxialSymmetricPt::FreeAxis::Z_AXIS enumeration constants.
//     *
//     */
//    //descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(/*[ COMPLETE HERE*/);
//
//    descartes_core::TrajectoryPtPtr pt = descartes_core::TrajectoryPtPtr(
//        new descartes_trajectory::AxialSymmetricPt(pose,ORIENTATION_INCREMENT,
//                                                   descartes_trajectory::AxialSymmetricPt::FreeAxis::Z_AXIS) );
//
//    // saving points into trajectory
//    traj.push_back(pt);
//  }


  // 1. Define sequence of points
  DescartesTrajectory points;

//  // for testing IK: move a verticle path
//  for (unsigned int i = 0; i < 15; ++i)
//  {
//      Eigen::Affine3d pose = descartes_core::utils::toFrame(
//              0.5f,              //x
//              0.0f,              //y
//              0.5f + 0.001 * i,  //z
//              0,                 //rx
//              M_PI,           //ry
//              0);                //rz
//      descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
//      points.push_back(pt);
//  }

  for (unsigned int i = 0; i < 15; ++i)
  {
    Eigen::Affine3d pose = descartes_core::utils::toFrame(  0.05f,              //x
                                                           -0.5f,               //y
                                                            0.35f + 0.001 * i,  //z
                                                            0,                 //rx
                                                            M_PI,              //ry
                                                            0);                 //rz
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
    points.push_back(pt);
  }

  Eigen::Vector3d pt1(0.05, -0.5, 0.365);
  Eigen::Vector3d pt2(0.052, -0.5, 0.371);
  Eigen::Vector3d pt3(0.06, -0.5, 0.375);
  Eigen::Vector3d center(0.06, -0.5, 0.365);
  buildCircleBy3Pt(pt1, pt2, pt3, center, points);

  for (unsigned int i = 0; i < 280; ++i)
  {
    Eigen::Affine3d pose = descartes_core::utils::toFrame(  0.06f + 0.001 * i, //x
                                                           -0.5f,              //y
                                                            0.375f,            //z
                                                            0,                 //rx
                                                            M_PI,              //ry
                                                            0);                //rz
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
    points.push_back(pt);
  }

  pt1 << 0.340, -0.5, 0.375;
  pt2 << 0.348, -0.5, 0.371;
  pt3 << 0.350, -0.5, 0.365;
  center << 0.340, -0.5, 0.365;
  buildCircleBy3Pt(pt1, pt2, pt3, center, points);

  for (unsigned int i = 0; i < 15; ++i)
  {
    Eigen::Affine3d pose = descartes_core::utils::toFrame(  pt3(0),             //x
                                                            pt3(1),             //y
                                                            pt3(2) - 0.001 * i, //z
                                                            0,                 //rx
                                                            M_PI,              //ry
                                                            0);                 //rz
    descartes_core::TrajectoryPtPtr pt = makeTolerancedCartesianPoint(pose);
    points.push_back(pt);
  }



  // 2. Create a robot model and initialize it
  descartes_core::RobotModelPtr model (new descartes_moveit::MoveitStateAdapter);

  // Name of description on parameter server. Typically just "robot_description".
  const std::string robot_description = "robot_description";

  // name of the kinematic group you defined when running MoveitSetupAssistant
  const std::string group_name = "arm1";

  // Name of frame in which you are expressing poses. Typically "world_frame" or "base_link".
  const std::string world_frame = "base_link";

  // tool center point frame (name of link associated with tool)
  // const std::string tcp_frame = "tool0";
  const std::string tcp_frame = "link6";

  if (!model->initialize(robot_description, group_name, world_frame, tcp_frame))
  {
    ROS_INFO("Could not initialize robot model");
    return -1;
  }

  // 3. Create a planner and initialize it with our robot model
  descartes_planner::DensePlanner planner;
  planner.initialize(model);

  // 4. Feed the trajectory to the planner
  if (!planner.planPath(points))
  {
    ROS_ERROR("Could not solve for a valid path");
    return -2;
  }

  DescartesTrajectory result;
  if (!planner.getPath(result))
  {
    ROS_ERROR("Could not retrieve path");
    return -3;
  }

  // 5. Translate the result into a type that ROS understands
  // Get Joint Names
  std::vector<std::string> names;
  nh.getParam("controller_joint_names", names);

#if 1
  // Generate a ROS joint trajectory with the result path, robot model, given joint names,
  // a certain time delta between each trajectory point
  trajectory_msgs::JointTrajectory joint_solution = toROSJointTrajectory(result, *model, names, 0); // the last param: time_delay for each point
//  trajectory_msgs::JointTrajectory joint_solution = toROSJointTrajectory(result, *model, names, 0.1); // the last param: time_delay for each point

  // 6. Send the ROS trajectory to the robot for execution
  if (!executeTrajectory(joint_solution))
  {
    ROS_ERROR("Could not execute trajectory!");
    return -4;
  }
#else
  runPath(result, model);
#endif

  ROS_INFO("Done!");
  // exiting ros node
  spinner.stop();

  return 0;
}

descartes_core::TrajectoryPtPtr makeCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;

  return TrajectoryPtPtr( new CartTrajectoryPt( TolerancedFrame(pose)) );
}

descartes_core::TrajectoryPtPtr makeTolerancedCartesianPoint(const Eigen::Affine3d& pose)
{
  using namespace descartes_core;
  using namespace descartes_trajectory;
  return TrajectoryPtPtr( new AxialSymmetricPt(pose, M_PI/2.0-0.0001, AxialSymmetricPt::Z_AXIS) );
}

trajectory_msgs::JointTrajectory
toROSJointTrajectory(const DescartesTrajectory& trajectory,
                     const descartes_core::RobotModel& model,
                     const std::vector<std::string>& joint_names,
                     double time_delay)
{
  // Fill out information about our trajectory
  trajectory_msgs::JointTrajectory result;
  result.header.stamp = ros::Time::now();
  result.header.frame_id = "world_frame";
  result.joint_names = joint_names;

  // For keeping track of time-so-far in the trajectory
  double time_offset = 0.0;
  // Loop through the trajectory
  for (TrajectoryIter it = trajectory.begin(); it != trajectory.end(); ++it)
  {
    // Find nominal joint solution at this point
    std::vector<double> joints;
    it->get()->getNominalJointPose(std::vector<double>(), model, joints);

    // Fill out a ROS trajectory point
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = joints;
    // velocity, acceleration, and effort are given dummy values
    // we'll let the controller figure them out
    pt.velocities.resize(joints.size(), 0.0);
    pt.accelerations.resize(joints.size(), 0.0);
    pt.effort.resize(joints.size(), 0.0);
    // set the time into the trajectory
    pt.time_from_start = ros::Duration(time_offset);
    // increment time
    time_offset += time_delay;

    result.points.push_back(pt);
  }

  return result;
}

const std::string PLANNER_ID = "RRTConnectkConfigDefault";

// use MoveIt to move the robot


void fromDescartesToMoveitTrajectory(const DescartesTrajectory& in_traj,
                                     trajectory_msgs::JointTrajectory& out_traj,
                                     const descartes_core::RobotModelPtr robot_model_ptr_)
{
  // Fill out information about our trajectory
  out_traj.header.stamp = ros::Time::now();
  out_traj.header.frame_id = config_.world_frame;
  out_traj.joint_names = config_.joint_names;

  // For keeping track of time-so-far in the trajectory
  double time_offset = 0.0;

  // Loop through the trajectory
  for (unsigned int i = 0; i < in_traj.size(); i++)
  {
    // Find nominal joint solution at this point
    std::vector<double> joints;

    // getting joint position at current point
    const descartes_core::TrajectoryPtPtr& joint_point = in_traj[i];
    joint_point->getNominalJointPose(std::vector<double>(), *robot_model_ptr_, joints);

    // Fill out a ROS trajectory point
    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = joints;
    // velocity, acceleration, and effort are given dummy values
    // we'll let the controller figure them out
    pt.velocities.resize(joints.size(), 0.0);
    pt.accelerations.resize(joints.size(), 0.0);
    pt.effort.resize(joints.size(), 0.0);
    // set the time into the trajectory
    pt.time_from_start = ros::Duration(time_offset);
    // increment time
    time_offset += config_.time_delay;

    out_traj.points.push_back(pt);
  }

}

void runPath(
  const DescartesTrajectory& path,
  const descartes_core::RobotModelPtr robot_model_ptr_ /* Performs tasks specific to the Robot, such as IK, FK and collision detection*/)
{
  // creating move group to move the arm in free space
  moveit::planning_interface::MoveGroup move_group("arm1");

  move_group.setPlannerId(PLANNER_ID);

  // creating goal joint pose to start of the path
  /*  Fill Code:
   * Goal:
   * - Retrieve the first point in the path.
   * - Save the joint values of the first point into "start_pose".
   * Hint:
   * - The first argument to "first_point_ptr->getNominalJointPose(...)" is a "std::vector<double>" variable
   *    where the joint values are to be stored.
   */
  std::vector<double> seed_pose(robot_model_ptr_->getDOF());
  std::vector<double> start_pose;


  //descartes_core::TrajectoryPtPtr first_point_ptr /* [ COMPLETE HERE ]: =  path[??]*/;
  /*[ COMPLETE HERE ]: first_point_ptr->getNominalJointPose(??,*robot_model_ptr_,start_pose); */
  descartes_core::TrajectoryPtPtr first_point_ptr = path[0];
  first_point_ptr->getNominalJointPose(seed_pose,*robot_model_ptr_,start_pose);

  // moving arm to joint goal
  move_group.setJointValueTarget(start_pose);
  move_group.setPlanningTime(10.0f);
  moveit_msgs::MoveItErrorCodes result = move_group.move();
  if(result.val != result.SUCCESS)
  {
    ROS_ERROR_STREAM("Move to start joint pose failed");
    exit(-1);
  }

  // creating Moveit trajectory from Descartes Trajectory
  moveit_msgs::RobotTrajectory moveit_traj;
  fromDescartesToMoveitTrajectory(path,moveit_traj.joint_trajectory, robot_model_ptr_);

  // sending robot path to server for execution
  /*  Fill Code:
   * Goal:
   * - Complete the service request by placing the "moveit_msgs::RobotTrajectory" trajectory in the request object
   * - Use the service client to send the trajectory for execution.
   * Hint:
   * - The "srv.request.trajectory" can be assigned the Moveit trajectory.
   * - The "moveit_run_path_client_.call(srv)" sends a trajectory execution request.
   */
  moveit_msgs::ExecuteKnownTrajectory srv;
  //srv.request.trajectory ; /* [ COMPLETE HERE ]: = ?? */;
  srv.request.trajectory = moveit_traj;
  srv.request.wait_for_execution = true;

  ROS_INFO_STREAM("Robot path sent for execution");
  if(moveit_run_path_client_.call(srv))
  {
    ROS_INFO_STREAM("Robot path execution completed");
  }
  else
  {
    ROS_ERROR_STREAM("Failed to run robot path with error "<<srv.response.error_code.val);
    exit(-1);
  }

  ROS_INFO_STREAM("Task '"<<__FUNCTION__<<"' completed");

}


bool executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Create a Follow Joint Trajectory Action Client
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("joint_trajectory_action", true);
  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR("Could not connect to action server");
    return false;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  goal.goal_time_tolerance = ros::Duration(1.0);
  
  ac.sendGoal(goal);

  if (ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start + ros::Duration(5)))
  {
    ROS_INFO("Action server reported successful execution");
    return true;
  } else {
    ROS_WARN("Action server could not execute trajectory");
    return false;
  }
}
