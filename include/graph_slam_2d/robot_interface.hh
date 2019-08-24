/**
 * Robot interface for the differential robot simulated in vrep with laser sensor 
 */
#ifndef ROBOT_INTERFACE_H_
#define ROBOT_INTERFACE_H_

#include "graph_slam_2d/common.hh"
#include "graph_slam_2d/laser_frame.hh"

#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Pose2D.h"

namespace graph_slam_2d{

class RobotInterface
{
  /**Robot State
   * Lost: RobotInterface find no robot to connect
   * Initialized: RobotInterface received all initialization message
   * Running: Robot is running and RobotInterface is receiving message from robot
   */
public:
  enum class State {Lost, Initialized, Running};
  //Initial state is lost, no connection
  State state_{State::Lost};
  bool running_flag = false;
  bool called_all_callback;
  
public:
  using Ptr = std::shared_ptr<RobotInterface>;

public:
  explicit RobotInterface(const ros::NodeHandle& nh);
  //Cannot copy
  RobotInterface(const RobotInterface& ) = delete;
  RobotInterface& operator=(const RobotInterface& ) = delete;
  ~RobotInterface(){
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old terminal settings
  };
public:
  /** 
   * Getters of robot configuration
   */ 
  //Circular robot diameter 
  double robotDiameter() const;
  //Two wheels' diameter
  double robotWheelDiameter() const;
  //Distance between two wheels
  double robotWheelDistance() const;
  
  /**
   *  Getters of robot state(2D pose)
   */
  Sophus::SE2 checkAndReturnPose(const Sophus::SE2& pose) const;
  void checkAndPublishPose(const Sophus::SE2& pose, Sophus::SE2& cache, const ros::Publisher& pub);
  //Transformation of Laser sensor frame in Robot reference frame
  Sophus::SE2 laserPoseInRobotFrame() const;
  //Robot's absolute pose in world frame
  Sophus::SE2 robotPose() const;
  //Robot's estimated pose in world frame
  Sophus::SE2 estimatedPose() const;
  void setEstimatedPose(const Sophus::SE2& estimated_pose);
  //Robot's target pose in world frame
  Sophus::SE2 targetPose() const;
  void setTargetPose(const Sophus::SE2& target_pose);
  
  //get and set visibility 
  bool estimatedPoseVisibility() const;
  void setEstimatedPoseVisibility(bool visible);
  bool targetPoseVisibility() const;
  void setTargetPoseVisibility(bool visible);
  /**
   * Getters of laser reading
   */
   std::shared_ptr<LaserFrame> getLaserFrame();
  
  /**
   * Robot command, given forward and angular velocity,
   * calculates right and left wheels' velocity and publish them
   */
  
  void setRobotVelocity(double v, double w);
  Eigen::Vector2d getWheelVel();
  Eigen::Vector2d getWheelJointValue();
  /**
   * Programe timing interface
   * stand alone timing function, don't use the time stamp in laser frame
   */
  double getElapsedTimeFromLastCall();
  void setVelocityFromKeyInput(const double& linear_scale, const double& angular_scale);
private:
  struct termios oldt,newt;
private:
  //ros NodeHandle
  ros::NodeHandle nh_;
  
  //robot configuration 
  float robot_diameter_{0};
  
  float robot_wheel_diameter_{0};
  
  float robot_wheel_distance_{0};
  
  enum SubsAndPubs{laser_scan,T_r_l,T_w_r,T_w_e,T_w_t,estimated_vis,target_vis,l_vel,r_vel,wheel_state};
  //laser frame in robot reference frame 
  std::string topic_name_[10];
  int queue_size_[10]={10,10,10,10,10,10,10,10,10,10};
  
  LaserFrame::Ptr laser_frame_ptr_;
  ros::Subscriber sub_laser_scan;
  bool valid_frame{false};
    
  //relative pose of laser frame in robot reference frame 
  Sophus::SE2 T_robot_laser_;
  ros::Subscriber sub_T_robot_laser_;
  
  //robot frame in world reference frame, ground truth  
  Sophus::SE2 T_world_robot_;
  ros::Subscriber sub_T_world_robot_;
  
  //estimated robot frame in world reference frame 
  Sophus::SE2 T_world_estimated_;
  //ros::Subscriber sub_T_world_estimated_;
  ros::Publisher pub_T_world_estimated_;
  
  //target frame in world reference frame 
  Sophus::SE2 T_world_target_;
  //ros::Subscriber sub_T_world_target_;
  ros::Publisher pub_T_world_target_;
  
  //visibility publisher 
  bool estimated_pose_visibility_{false};
  ros::Publisher pub_estimated_pose_visibility_;
  
  bool target_pose_visibility_{false};
  ros::Publisher pub_target_pose_visibility_;
  
  //subscriber and publisher of control velocity 
  //publisher only publish the velocity to topic, but does not change the internal cache
  //subscriber will change the internal cache, it record the true velocity 
  //ATTENTION: 
  //DO NOT use joint velocity multiply elapsed time to get input, the clock frequency in simulator is
  //different from ROS node, need to use the current joint value minus the past joint value 
  Eigen::Vector2d wheel_velocity_; //[left,right]
  Eigen::Vector2d wheel_joint_value_;//[left,right]
  ros::Publisher pub_left_wheel_vel_;
  ros::Publisher pub_right_wheel_vel_;
  ros::Subscriber sub_wheel_state_;

private:
  //Callback functions for laser scan, convert LaserScan message to 2D coordinate 
  void laser_scan_callback(const sensor_msgs::LaserScan& msg);
  void write_scan_to_file();
  
  //Callback functions for robot state
  void T_rl_callback(const geometry_msgs::Pose2D& msg);
  void T_wr_callback(const geometry_msgs::Pose2D& msg);
  //void T_we_callback(const geometry_msgs::Pose2D& msg);
  //void T_wt_callback(const geometry_msgs::Pose2D& msg);
  void wheel_state_callback(const sensor_msgs::JointState& msg); 
  //convertion between Pose2D and SE2 
  inline Eigen::Vector3d convertPose2DToVector(const geometry_msgs::Pose2D& msg);
  inline geometry_msgs::Pose2D convertVectorToPose2D(const Eigen::Vector3d& s);
  inline Sophus::SE2 convertPose2DToSE2(const geometry_msgs::Pose2D& msg);
  inline geometry_msgs::Pose2D convertSE2ToPose2D(const Sophus::SE2& T);
};
Eigen::Vector3d RobotInterface::convertPose2DToVector(const geometry_msgs::Pose2D& msg)
{
  return Eigen::Vector3d(msg.x,msg.y,msg.theta);
}

geometry_msgs::Pose2D RobotInterface::convertVectorToPose2D(const Eigen::Vector3d& s)
{
  geometry_msgs::Pose2D pose2D;
  pose2D.x = s(0);
  pose2D.y = s(1);
  pose2D.theta = s(2);
  return pose2D;
}

Sophus::SE2 RobotInterface::convertPose2DToSE2(const geometry_msgs::Pose2D& msg)
{
  return Sophus::SE2(msg.theta, Eigen::Vector2d(msg.x, msg.y));
}

geometry_msgs::Pose2D RobotInterface::convertSE2ToPose2D(const Sophus::SE2& T)
{
  geometry_msgs::Pose2D pose2D;
  pose2D.x = T.translation()(0);
  pose2D.y = T.translation()(1);
  pose2D.theta = T.so2().log();
}



}

#endif //ROBOT_INTERFACE_H_
