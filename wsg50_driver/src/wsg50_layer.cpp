// #include <signal.h>
// #include <stdio.h>
#include <stdlib.h>
// #include <string.h>
// #include <assert.h>
// #include <thread>
// #include <chrono>
// 
#include "wsg50/common.h"
#include "wsg50/cmd.h"
#include "wsg50/msg.h"
#include "wsg50/functions.h"
// 
#include <ros/ros.h>
#include "std_srvs/Empty.h"
#include "std_msgs/String.h"
#include "wsg50_common/Status.h"
#include "wsg50_common/Move.h"
#include "wsg50_common/Conf.h"
#include "wsg50_common/Incr.h"
#include "wsg50_common/Cmd.h"
// 
// #include "sensor_msgs/JointState.h"
// #include "std_msgs/Float32.h"
// #include "std_msgs/Bool.h"

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <controller_manager/controller_manager.h>
#include <boost/shared_ptr.hpp>
#include <boost/graph/graph_concepts.hpp>
#include <hardware_interface/posvel_command_interface.h>
#include <hardware_interface/wsg50_grasping_interface.h>

#define GRIPPER_MIN_OPEN 0.0


class WSG50Layer : public hardware_interface::RobotHW
{
private:
  ros::NodeHandle nh_;
  std::string ip, protocol, com_mode, serial_port;
  int size, port, local_port, serial_baudrate;
  double rate;
  std::string joint_name_;
  double joint_position_;
  double joint_speed_;
  double joint_effort_;
  double desired_target_position_;
  double actual_target_position_;  
  double desired_target_speed_;
  double actual_target_speed_;
  int gripper_size;
  
  bool isInitialized_;
  double actual_force_limit_;
  double desired_force_limit_;
  bool must_grasp_;
  bool objectGraspped;
  
  
  ros::Time lastUpdateTime;
  
  ros::ServiceServer releaseSS, homingSS, stopSS, ackSS, incrementSS, setAccSS;
  
  hardware_interface::JointStateInterface joint_state_interface;
  hardware_interface::PosVelJointInterface pos_vel_interface;
  hardware_interface::WSG50GraspingInterface grasping_interface;
  
public:
  WSG50Layer() 
  {
    nh_ = ros::NodeHandle("~");
  }
  ~WSG50Layer() {nh_.shutdown();}
  
  bool init()
  {
    nh_.param("size", size, 210);
    nh_.param("ip", ip, std::string("192.168.1.20"));
    nh_.param("port", port, 1000);
    nh_.param("local_port", local_port, 1501);
    nh_.param("serial_port", serial_port, std::string("/dev/ttyS1"));
    nh_.param("serial_baudrate", serial_baudrate, 115200);
    nh_.param("protocol", protocol, std::string("serial"));
    nh_.param("com_mode", com_mode, std::string("pooling"));
    nh_.param("rate", rate, 5.0); // With custom script, up to 30Hz are possible
    nh_.param("grasping_force", desired_force_limit_, 0.0);

    if (size != 210 or size != 110) {
	ROS_WARN("Gripper size: %d is invalid. Using default size of 210 mm.", size);
	gripper_size = 210;
    }
    else {
	gripper_size = size;
    }
    
    // must be in parameters
    joint_name_ = "wsg50_finger_left_joint";

    int res_con = -1;
    if (protocol == "serial") {
        res_con = cmd_connect_serial(serial_port.c_str(), serial_baudrate);
        ROS_INFO("Connecting to %s:%d (%s); communication mode: %s ...", serial_port.c_str(), serial_baudrate, protocol.c_str(), com_mode.c_str());
    }
    else if (protocol == "udp") {
        res_con = cmd_connect_udp(local_port, ip.c_str(), port );
        ROS_INFO("Connecting to %s:%d (%s); communication mode: %s ...", ip.c_str(), port, protocol.c_str(), com_mode.c_str());
    }
    else if (protocol == "tcp") {
        res_con = cmd_connect_tcp( ip.c_str(), port );
        ROS_INFO("Connecting to %s:%d (%s); communication mode: %s ...", ip.c_str(), port, protocol.c_str(), com_mode.c_str());
    }
    else {
        ROS_ERROR("UNKNOWN protocol!");
    }
    
    if (res_con != 0 ) {
      ROS_ERROR("Unable to connect, please check the port and address used.");
      nh_.shutdown();
      return false;
    }
    
    ROS_INFO("Gripper connection established");

    // Services
    incrementSS = nh_.advertiseService("move_incrementally", &WSG50Layer::incrementSrv, this);
    releaseSS = nh_.advertiseService("release", &WSG50Layer::releaseSrv, this);
    homingSS = nh_.advertiseService("homing", &WSG50Layer::homingSrv, this);
    stopSS = nh_.advertiseService("stop", &WSG50Layer::stopSrv, this);
    ackSS = nh_.advertiseService("ack", &WSG50Layer::ackSrv, this);
    setAccSS = nh_.advertiseService("set_acceleration", &WSG50Layer::setAccSrv, this);

    ROS_INFO("Ready to use, homing now...");
    homing();
    sleep(4);

    if (desired_force_limit_ > 0.0) {
	ROS_INFO("Setting grasping force limit to %5.1f", desired_force_limit_);
	setGraspingForceLimit(desired_force_limit_);
	actual_force_limit_ = desired_force_limit_;
    }
    
    initHardwareInterface();
    must_grasp_ = false;
    
    
    actual_target_position_ = getOpening();
    desired_target_position_ = actual_target_position_;
    actual_target_speed_ = 0.0;
    desired_target_speed_ = actual_target_speed_;
    
    isInitialized_ = true;
    ROS_INFO("Init done.");
    return true;
  }

  void initHardwareInterface()
  {
    //Initialize hardware interface for ros control
    ROS_INFO("Initialize hardware interface for joint: %s", joint_name_.c_str());
    hardware_interface::JointStateHandle state_handle(joint_name_, &joint_position_, 
						      &joint_speed_, &joint_effort_);
    joint_state_interface.registerHandle(state_handle);
    
    hardware_interface::PosVelJointHandle pos_vel_handle(state_handle, &desired_target_position_,
	&desired_target_speed_);
    pos_vel_interface.registerHandle(pos_vel_handle);
    
    hardware_interface::WSG50GraspingHandle grasping_handle(joint_name_, &desired_target_position_,
	&desired_target_speed_, &desired_force_limit_, &must_grasp_);
    grasping_interface.registerHandle(grasping_handle);
    
    
    registerInterface(&joint_state_interface);
    registerInterface(&pos_vel_interface);
    registerInterface(&grasping_interface);
  }
  
  void read(const ros::Time& time, const ros::Duration& period)
  {
    joint_position_ = getOpening();
    ros::Duration d = ros::Time::now() - lastUpdateTime;
    double fr = 1 / rate;
    if (d.toSec() > fr)
    {
      joint_effort_ = getForce();
      lastUpdateTime = ros::Time::now();
    }
  }
  
  void write(const ros::Time& time, const ros::Duration& period)
  {
    if (!isInitialized_) {
	ROS_DEBUG("wsg50 is not initialized!");
	return;
    }
    
    if (actual_target_position_ == desired_target_position_) { return; }
    
    actual_target_position_ = desired_target_position_;
    if (actual_target_speed_ != desired_target_speed_) 
    {
      actual_target_speed_ = desired_target_speed_;
    }
    
    if (must_grasp_)
    {
      if (actual_force_limit_ != desired_force_limit_)
      {
	actual_force_limit_ = desired_force_limit_;
	setGraspingForceLimit(desired_force_limit_);
      }
      graspSrv(desired_target_position_, desired_target_speed_);
    }
    else
    {
      moveSrv(desired_target_position_, desired_target_speed_);
    }
    
  }
  
  void moveSrv(double width, double speed)
  {
	  if ( (width >= 0.0 && width <= gripper_size) && (speed > 0.0 && speed <= 420.0) ){
		  ROS_INFO("Moving to %f position at %f mm/s.", width, speed);
		  move(width, speed, false);
	  }else if (width < 0.0 || width > gripper_size){
		  ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - %d] ", gripper_size);
		  return;
	  }else{
		  ROS_WARN("Speed values are outside the gripper's physical limits ([0.1 - 420.0])  Using clamped values.");
		  move(width, speed, false);
	  }
	  ROS_INFO("Target position reached.");
  }

  void graspSrv(double width, double speed)
  {
	  if ( (width >= 0.0 && width <= gripper_size) && (speed > 0.0 && speed <= 420.0) ){
	  ROS_INFO("Grasping object at %f with %f mm/s.", width, speed);
		  grasp(width, speed);
	  }else if (width < 0.0 || width > gripper_size){
		  ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - %d] ", gripper_size);
		  return;
	  }else{
		  ROS_WARN("Speed or position values are outside the gripper's physical limits (Position: [0.0 - %d] / Speed: [0.1 - 420.0])  Using clamped values.", gripper_size);
		  grasp(width, speed);
	  }

	  ROS_INFO("Object grasped correctly.");
	  objectGraspped = true;
  }

  bool incrementSrv(wsg50_common::Incr::Request &req, wsg50_common::Incr::Response &res)
  {
	  if (req.direction == "open"){
	  
		  if (!objectGraspped){
		  
			  float currentWidth = getOpening();
			  float nextWidth = currentWidth + req.increment;
			  if ( (currentWidth < gripper_size) && nextWidth < gripper_size ){
				  //grasp(nextWidth, 1);
				  move(nextWidth,20, true);
				  currentWidth = nextWidth;
			  }else if( nextWidth >= gripper_size){
				  //grasp(gripper_size, 1);
				  move(gripper_size,1, true);
				  currentWidth = gripper_size;
			  }
		  }else{
			  ROS_INFO("Releasing object...");
			  release(gripper_size, 20);
			  objectGraspped = false;
		  }
	  }else if (req.direction == "close"){
	  
		  if (!objectGraspped){

			  float currentWidth = getOpening();
			  float nextWidth = currentWidth - req.increment;
		  
			  if ( (currentWidth > GRIPPER_MIN_OPEN) && nextWidth > GRIPPER_MIN_OPEN ){
				  //grasp(nextWidth, 1);
				  move(nextWidth,20, true);
				  currentWidth = nextWidth;
			  }else if( nextWidth <= GRIPPER_MIN_OPEN){
				  //grasp(GRIPPER_MIN_OPEN, 1);
				  move(GRIPPER_MIN_OPEN,1, true);
				  currentWidth = GRIPPER_MIN_OPEN;
			  }
		  }
	  }
	  return true;
  }

  bool releaseSrv(wsg50_common::Move::Request &req, wsg50_common::Move::Response &res)
  {
	  if ( (req.width >= 0.0 && req.width <= gripper_size) && (req.speed > 0.0 && req.speed <= 420.0) ){
		  ROS_INFO("Releasing to %f position at %f mm/s.", req.width, req.speed);
		  res.error = release(req.width, req.speed);
	  }else if (req.width < 0.0 || req.width > gripper_size){
		  ROS_ERROR("Imposible to move to this position. (Width values: [0.0 - gripper_size] ");
		  res.error = 255;
		  return false;
	  }else{
		  ROS_WARN("Speed or position values are outside the gripper's physical limits (Position: [0.0 - gripper_size] / Speed: [0.1 - 420.0])  Using clamped values.");
		  res.error = release(req.width, req.speed);
	  }
	  ROS_INFO("Object released correctly.");
	  return true;
  }	

  bool homingSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
  {
	  ROS_INFO("Homing...");
	  homing();
	  ROS_INFO("Home position reached.");
	  return true;
  }

  bool stopSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
  {
	  ROS_WARN("Stop!");
	  stop();
	  ROS_WARN("Stopped.");
	  return true;
  }

  bool setAccSrv(wsg50_common::Conf::Request &req, wsg50_common::Conf::Response &res)
  {
	  setAcceleration(req.val);
	  return true;
  }

  bool ackSrv(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
  {
	  ack_fault();
	  return true;
  }
    
};


int main(int argc, char** argv)
{
  // initialize ROS, spezify name of node
  ros::init(argc, argv, "wsg50layer");
  
  ros::NodeHandle nh("~");
  WSG50Layer wsg50_node;
  bool isInitialized = wsg50_node.init();

  if (!isInitialized)
    return 0;

  ROS_INFO("...wsg50 node running...");
  
  controller_manager::ControllerManager cm(&wsg50_node, nh);
  ROS_INFO("...cm running...");
  ros::AsyncSpinner spinner(2); // Use 2 threads
  spinner.start();
  
  ros::Time time;
  ros::Duration period(0.1);
  while (ros::ok()) { 
      time = ros::Time::now();
      wsg50_node.read(time, period);
      cm.update(time, period);
      wsg50_node.write(time, period);
      ros::spinOnce();
      period.sleep();
  }
  spinner.stop();

  return 0;
}