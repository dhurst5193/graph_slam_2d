#include "graph_slam_2d/robot_interface.hh"
#include "graph_slam_2d/laser_frame.hh"
#include "graph_slam_2d/scan_matcher.hh"
using graph_slam_2d::LaserFrame;
using graph_slam_2d::RobotInterface;
using graph_slam_2d::ScanMatcher;
int main(int argc, char** argv){
  //ros initiation 
  ros::init(argc, argv, "ekf slam");
  ros::NodeHandle nh("~");
  ros::Rate loopRate(20);
  //create a robot interface 
  RobotInterface::Ptr robot = std::make_shared<RobotInterface>(nh);
  //get robot parameter 
  double wheel_distance = robot->robotWheelDistance();
  double wheel_radius = robot->robotWheelDiameter()/2;
  
  std::shared_ptr<ScanMatcher> matcher_ptr = std::make_shared<ScanMatcher>();
  
  //spin loop 
  LaserFrame::Ptr frame_curr;
  LaserFrame::Ptr frame_past;
  Eigen::Vector2d joint_curr;
  Eigen::Vector2d joint_past;
  size_t iterations  = 0;
  Sophus::SE2 T_matched(0,Eigen::Vector2d(0,0));
  while(ros::ok()){
    if(robot->state_ == RobotInterface::State::Running){
      //Read a new frame
      if(iterations == 0){
	//initialization
	frame_past = robot->getLaserFrame();
	joint_past = robot->getWheelJointValue();
      } else{
	frame_curr = robot->getLaserFrame();
	joint_curr = robot->getWheelJointValue();
	//Compute the wheel odometry, relative transformation prediction
	Eigen::Vector2d u = (joint_curr - joint_past)*wheel_radius;
	Eigen::Vector3d se2_odom;
	 double theta = T_matched.so2().log();
	se2_odom(0) = (u(0)+u(1))/2 * cos(theta +(u(1)-u(0))/(2*wheel_distance));
	se2_odom(1) = (u(0)+u(1))/2 * sin(theta +(u(1)-u(0))/(2*wheel_distance));
	se2_odom(2) = (u(1)-u(0))/wheel_distance;
	//Compute relative transformation from ScanMatcher
	PointMatcher<double>::DataPoints query_points(frame_curr->laser_points_, PointMatcher<double>::DataPoints::Labels());
	PointMatcher<double>::DataPoints reference_points(frame_past->laser_points_, PointMatcher<double>::DataPoints::Labels());

	auto match_result = matcher_ptr->run_icp_matching(query_points, reference_points, 
				    Sophus::SE2(se2_odom(2), Eigen::Vector2d(se2_odom(0), se2_odom(1))));
	//Update
	T_matched *= match_result.first;
	frame_past = frame_past;
	joint_past = joint_curr;
	//Show ICP result
	robot->setEstimatedPose(T_matched * match_result.first);
      }
    }
    //ros spin 
    ros::spinOnce();
    loopRate.sleep();
  }
}
