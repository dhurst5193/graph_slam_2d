#ifndef LASER_FRAME_H_
#define LASER_FRAME_H_

#include "graph_slam_2d/common.hh"
namespace graph_slam_2d{
class LaserFrame{
public: 
  using Ptr = std::shared_ptr<LaserFrame>;
  unsigned long id_; //laser reading id 
  double time_stamp_; //laser reading frame time stamp
  Eigen::Matrix<double,3,Eigen::Dynamic> laser_points_; //laser reading, represented as point in laser frame 
  //Vector3d T_w_l_; //laser sensor's pose in world frame 
public:
  //LaserFrame(const unsigned long& id, const double& time_stamp=0, const vector<Vector2d>& laser_points=vector<Vector2d>(), const Vector3d& T_w_l = Vector3d());
  LaserFrame(const unsigned long& id, const double& time_stamp=0, const Eigen::Matrix<double,3,Eigen::Dynamic>& laser_points=Eigen::Matrix<double,3,Eigen::Dynamic>());
};
}
#endif
