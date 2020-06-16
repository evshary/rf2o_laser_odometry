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

namespace rf2o {

class CLaserOdometry2DNode : CLaserOdometry2D
{
public:

  CLaserOdometry2DNode(rclcpp::Node::SharedPtr);
  ~CLaserOdometry2DNode() = default;

#if 0
  void process(const ros::TimerEvent &);
  void publish();

  bool setLaserPoseFromTf();

public:
  sensor_msgs::msg::LaserScan      last_scan;
  bool                        GT_pose_initialized;
  tf::TransformListener       tf_listener;          //Do not put inside the callback
  tf::TransformBroadcaster    odom_broadcaster;
  nav_msgs::msg::Odometry          initial_robot_pose;

  //Subscriptions & Publishers
  ros::Subscriber laser_sub, initPose_sub;
  ros::Publisher odom_pub;

  bool scan_available();

  //CallBacks
  void LaserCallBack(const sensor_msgs::msg::LaserScan::ConstPtr& new_scan);
  void initPoseCallBack(const nav_msgs::msg::Odometry::ConstPtr& new_initPose);
#endif
private:
  bool publish_tf, new_scan_available;
  double freq;

  std::string laser_scan_topic;
  std::string odom_topic;
  std::string base_frame_id;
  std::string odom_frame_id;
  std::string init_pose_from_topic;

  rclcpp::Node::SharedPtr node_;
};

CLaserOdometry2DNode::CLaserOdometry2DNode(rclcpp::Node::SharedPtr nh) :
  CLaserOdometry2D()
{
  node_ = nh;

#if 0
  RCLCPP_INFO("Initializing RF2O node...");
#endif

  //Read Parameters
  //----------------
  this->laser_scan_topic = node_->declare_parameter("laser_scan_topic", "/laser_scan");
  this->odom_topic = node_->declare_parameter("odom_topic", "/odom_rf2o");
  this->base_frame_id = node_->declare_parameter("base_frame_id", "/base_link");
  this->odom_frame_id = node_->declare_parameter("odom_frame_id", "/odom");
  this->init_pose_from_topic = node_->declare_parameter("init_pose_from_topic", "/base_pose_ground_truth");
  this->publish_tf = node_->declare_parameter("publish_tf", true);
  this->freq = node_->declare_parameter("freq", 10.0);
  this->verbose = node_->declare_parameter("verbose", true);

#if 0
  ros::NodeHandle pn("~");
  pn.param<std::string>("laser_scan_topic",laser_scan_topic,"/laser_scan");
  pn.param<std::string>("odom_topic", odom_topic, "/odom_rf2o");
  pn.param<std::string>("base_frame_id", base_frame_id, "/base_link");
  pn.param<std::string>("odom_frame_id", odom_frame_id, "/odom");
  pn.param<bool>("publish_tf", publish_tf, true);
  pn.param<std::string>("init_pose_from_topic", init_pose_from_topic, "/base_pose_ground_truth");
  pn.param<double>("freq",freq,10.0);
  pn.param<bool>("verbose", verbose, true);
#endif

#if 0
  //Publishers and Subscribers
  //--------------------------
  odom_pub  = pn.advertise<nav_msgs::msg::Odometry>(odom_topic, 5);
  laser_sub = n.subscribe<sensor_msgs::msg::LaserScan>(laser_scan_topic,1,&CLaserOdometry2DNode::LaserCallBack,this);

  //init pose??
  if (init_pose_from_topic != "")
  {
    initPose_sub = n.subscribe<nav_msgs::msg::Odometry>(init_pose_from_topic,1,&CLaserOdometry2DNode::initPoseCallBack,this);
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
#endif

#if 0
  RCLCPP_INFO_STREAM("Listening laser scan from topic: " << laser_sub.getTopic());
#endif
}

#if 0
bool CLaserOdometry2DNode::setLaserPoseFromTf()
{
  bool retrieved = false;

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
#if 0
    RCLCPP_ERROR("%s",ex.what());
#endif
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

  return retrieved;
}

bool CLaserOdometry2DNode::scan_available()
{
  return new_scan_available;
}

void CLaserOdometry2DNode::process(const ros::TimerEvent&)
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
#if 0
    RCLCPP_WARN("Waiting for laser_scans....") ;
#endif
  }
}

//-----------------------------------------------------------------------------------
//                                   CALLBACKS
//-----------------------------------------------------------------------------------

void CLaserOdometry2DNode::LaserCallBack(const sensor_msgs::msg::LaserScan::ConstPtr& new_scan)
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

void CLaserOdometry2DNode::initPoseCallBack(const nav_msgs::msg::Odometry::ConstPtr& new_initPose)
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
    //RCLCPP_INFO("[rf2o] Publishing TF: [base_link] to [odom]");
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = last_odom_time;
    odom_trans.header.frame_id = odom_frame_id;
    odom_trans.child_frame_id = base_frame_id;
    odom_trans.transform.translation.x = robot_pose_.translation()(0);
    odom_trans.transform.translation.y = robot_pose_.translation()(1);
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(rf2o::getYaw(robot_pose_.rotation()));
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);
  }

  //next, we'll publish the odometry message over ROS
  //-------------------------------------------------
  //RCLCPP_INFO("[rf2o] Publishing Odom Topic");
  nav_msgs::msg::Odometry odom;
  odom.header.stamp = last_odom_time;
  odom.header.frame_id = odom_frame_id;
  //set the position
  odom.pose.pose.position.x = robot_pose_.translation()(0);
  odom.pose.pose.position.y = robot_pose_.translation()(1);
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(rf2o::getYaw(robot_pose_.rotation()));
  //set the velocity
  odom.child_frame_id = base_frame_id;
  odom.twist.twist.linear.x = lin_speed;    //linear speed
  odom.twist.twist.linear.y = 0.0;
  odom.twist.twist.angular.z = ang_speed;   //angular speed
  //publish the message
  odom_pub.publish(odom);
}
#endif

} /* namespace rf2o */

//-----------------------------------------------------------------------------------
//                                   MAIN
//-----------------------------------------------------------------------------------
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto nh = rclcpp::Node::make_shared("RF2O_LaserOdom");

  rf2o::CLaserOdometry2DNode myLaserOdomNode(nh);

#if 0
  ros::init(argc, argv, "RF2O_LaserOdom");

  rf2o::CLaserOdometry2DNode myLaserOdomNode;

  ros::TimerOptions timer_opt;
  timer_opt.oneshot   = false;
  timer_opt.autostart = true;
  timer_opt.callback_queue = ros::getGlobalCallbackQueue();
  timer_opt.tracked_object = ros::VoidConstPtr();

  timer_opt.callback = boost::bind(&rf2o::CLaserOdometry2DNode::process, &myLaserOdomNode, _1);
  timer_opt.period   = ros::Rate(myLaserOdomNode.freq).expectedCycleTime();

  ros::Timer rf2o_timer = ros::NodeHandle("~").createTimer(timer_opt);

  ros::spin();
#endif

  return EXIT_SUCCESS;
}
