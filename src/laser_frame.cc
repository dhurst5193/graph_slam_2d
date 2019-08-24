#include "graph_slam_2d/common.hh"
#include "graph_slam_2d/laser_frame.hh"

namespace graph_slam_2d{
//LaserFrame::LaserFrame(const long unsigned int& id, const double& time_stamp, const vector< Vector2d >& laser_points, const Vector3d& T_w_l)
//: id_(id),time_stamp_(time_stamp),laser_points_(laser_points),T_w_l_(T_w_l) {}
LaserFrame::LaserFrame(const long unsigned int& id, const double& time_stamp, const Eigen::Matrix<double,3,Eigen::Dynamic>& laser_points)
: id_(id),time_stamp_(time_stamp),laser_points_(laser_points){}


}

