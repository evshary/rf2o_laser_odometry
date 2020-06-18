/** ****************************************************************************************
*  This node presents a fast and precise method to estimate the planar motion of a lidar
*  from consecutive range scans. It is very useful for the estimation of the robot odometry from
*  2D laser range measurements.
*  This module is developed for mobile robots with innacurate or inexistent built-in odometry.
*  It allows the estimation of a precise odometry with low computational cost.
*  For more information, please refer to:
*
*  Planar Odometry from a Radial Laser Scanner. A Range Flow-based Approach. ICRA'16.
*  Available at: http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/papers/217
*
* Maintainer: Javier G. Monroy
* MAPIR group: http://mapir.isa.uma.es/
*
* Modifications: Jeremie Deray
******************************************************************************************** */

#include "rf2o_laser_odometry/CLaserOdometry2D.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
using namespace std::chrono_literals;

namespace rf2o {

class CLaserOdometry2DNode : CLaserOdometry2D, public rclcpp::Node
{
public:

  CLaserOdometry2DNode();
  ~CLaserOdometry2DNode() = default;

  void publish();
  bool setLaserPoseFromTf();

private:
#if 0
  // TF
  tf::TransformListener       tf_listener;          //Do not put inside the callback
  tf::TransformBroadcaster    odom_broadcaster;
#endif

  bool scan_available();
  bool new_scan_available;

  // CallBacks
  void LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan);
  void initPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr new_initPose);
  void process();
  // variable used by callbacks
  sensor_msgs::msg::LaserScan last_scan;
  bool GT_pose_initialized;
  nav_msgs::msg::Odometry initial_robot_pose;

  // parameters
  bool publish_tf;
  double freq;
  std::string laser_scan_topic;
  std::string odom_topic;
  std::string base_frame_id;
  std::string odom_frame_id;
  std::string init_pose_from_topic;

  // Subscriptions & Publishers & timer
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr initPose_sub;
  rclcpp::TimerBase::SharedPtr timer_;
};

CLaserOdometry2DNode::CLaserOdometry2DNode() :
  CLaserOdometry2D(),
  Node("RF2O_LaserOdom")
{
  RCLCPP_INFO(this->get_logger(), "Initializing RF2O node...");

  //Read Parameters
  //----------------
  this->laser_scan_topic = this->declare_parameter("laser_scan_topic", "/laser_scan");
  this->odom_topic = this->declare_parameter("odom_topic", "/odom_rf2o");
  this->base_frame_id = this->declare_parameter("base_frame_id", "/base_link");
  this->odom_frame_id = this->declare_parameter("odom_frame_id", "/odom");
  this->init_pose_from_topic = this->declare_parameter("init_pose_from_topic", "/base_pose_ground_truth");
  this->publish_tf = this->declare_parameter("publish_tf", true);
  this->freq = this->declare_parameter("freq", 10.0);
  this->verbose = this->declare_parameter("verbose", true);

  //Publishers and Subscribers
  //--------------------------
  rclcpp::QoS qos(rclcpp::KeepLast(5));
  odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, qos);
  laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(laser_scan_topic, 10, std::bind(&CLaserOdometry2DNode::LaserCallBack, this, std::placeholders::_1));
  std::chrono::duration<double, std::ratio<1, 1000000>> us_period((1000000/this->freq));
  timer_ = create_wall_timer(us_period, std::bind(&CLaserOdometry2DNode::process, this));

  //init pose??
  if (init_pose_from_topic != "")
  {
    initPose_sub = this->create_subscription<nav_msgs::msg::Odometry>(init_pose_from_topic, 1, std::bind(&CLaserOdometry2DNode::initPoseCallBack, this, std::placeholders::_1));
    GT_pose_initialized  = false;
  }
  else
  {
    GT_pose_initialized = true;
    initial_robot_pose.pose.pose.position.x = 0;
    initial_robot_pose.pose.pose.position.y = 0;
    initial_robot_pose.pose.pose.position.z = 0;
    initial_robot_pose.pose.pose.orientation.w = 0;
    initial_robot_pose.pose.pose.orientation.x = 0;
    initial_robot_pose.pose.pose.orientation.y = 0;
    initial_robot_pose.pose.pose.orientation.z = 0;
  }

  setLaserPoseFromTf();

  //Init variables
  module_initialized = false;
  first_laser_scan   = true;

  RCLCPP_INFO_STREAM(this->get_logger(), "Listening laser scan from topic: " << laser_sub->get_topic_name());
}

bool CLaserOdometry2DNode::setLaserPoseFromTf()
{
  bool retrieved = false;

#if 0
  // Set laser pose on the robot (through tF)
  // This allow estimation of the odometry with respect to the robot base reference system.
  tf::StampedTransform transform;
  transform.setIdentity();
  try
  {
    tf_listener.lookupTransform(base_frame_id, last_scan.header.frame_id, ros::Time(0), transform);
    retrieved = true;
  }
  catch (tf::TransformException &ex)
  {
    RCLCPP_ERROR(this->get_logger(), "%s",ex.what());
    ros::Duration(1.0).sleep();
    retrieved = false;
  }

  //TF:transform -> Eigen::Isometry3d

  const tf::Matrix3x3 &basis = transform.getBasis();
  Eigen::Matrix3d R;

  for(int r = 0; r < 3; r++)
    for(int c = 0; c < 3; c++)
      R(r,c) = basis[r][c];

  Pose3d laser_tf(R);

  const tf::Vector3 &t = transform.getOrigin();
  laser_tf.translation()(0) = t[0];
  laser_tf.translation()(1) = t[1];
  laser_tf.translation()(2) = t[2];

  setLaserPose(laser_tf);
#endif

  return retrieved;
}

bool CLaserOdometry2DNode::scan_available()
{
  return new_scan_available;
}

void CLaserOdometry2DNode::process()
{
  if( is_initialized() && scan_available() )
  {
    //Process odometry estimation
    odometryCalculation(last_scan);
    publish();
    new_scan_available = false; //avoids the possibility to run twice on the same laser scan
  }
  else
  {
    RCLCPP_WARN(this->get_logger(), "Waiting for laser_scans....") ;
  }
}

//-----------------------------------------------------------------------------------
//                                   CALLBACKS
//-----------------------------------------------------------------------------------

void CLaserOdometry2DNode::LaserCallBack(const sensor_msgs::msg::LaserScan::SharedPtr new_scan)
{
  if (GT_pose_initialized)
  {
    //Keep in memory the last received laser_scan
    last_scan = *new_scan;
    current_scan_time = last_scan.header.stamp;

    //Initialize module on first scan
    if (!first_laser_scan)
    {
      //copy laser scan to internal variable
      for (unsigned int i = 0; i<width; i++)
        range_wf(i) = new_scan->ranges[i];
      new_scan_available = true;
    }
    else
    {
      init(last_scan, initial_robot_pose.pose.pose);
      first_laser_scan = false;
    }
  }
}

void CLaserOdometry2DNode::initPoseCallBack(const nav_msgs::msg::Odometry::SharedPtr new_initPose)
{
  //Initialize module on first GT pose. Else do Nothing!
  if (!GT_pose_initialized)
  {
    initial_robot_pose = *new_initPose;
    GT_pose_initialized = true;
  }
}

void CLaserOdometry2DNode::publish()
{
  //first, we'll publish the odometry over tf
  //---------------------------------------
  if (publish_tf)
  {
    RCLCPP_INFO(this->get_logger(), "[rf2o] Publishing TF: [base_link] to [odom]");
    geometry_msgs::msg::TransformStamped odom_trans;
    odom_trans.header.stamp = last_odom_time;
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;
    odom_trans.transform.translation.x = robot_pose_.translation()(0);
    odom_trans.transform.translation.y = robot_pose_.translation()(1);
    odom_trans.transform.translation.z = 0.0;
    //odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(rf2o::getYaw(robot_pose_.rotation()));
    //send the transform
    //odom_broadcaster.sendTransform(odom_trans);
  }

  //next, we'll publish the odometry message over ROS
  //-------------------------------------------------
  RCLCPP_INFO(this->get_logger(), "[rf2o] Publishing Odom Topic");
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = last_odom_time;
  odom.header.frame_id = odom_frame_id;
  //set the position
  odom.pose.pose.position.x = robot_pose_.translation()(0);
  odom.pose.pose.position.y = robot_pose_.translation()(1);
  odom.pose.pose.position.z = 0.0;
  //odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(rf2o::getYaw(robot_pose_.rotation()));
  //set the velocity
  odom.child_frame_id = base_frame_id;
  odom.twist.twist.linear.x = lin_speed;    //linear speed
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = ang_speed;   //angular speed
  //publish the message
  odom_pub->publish(odom);
}

} /* namespace rf2o */

//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<rf2o::CLaserOdometry2DNode>());
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
