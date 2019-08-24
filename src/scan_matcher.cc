#include "graph_slam_2d/scan_matcher.hh"

namespace graph_slam_2d {
  
ScanMatcher::ScanMatcher(): icp_()
{
  icp_.setDefault();
}
  
std::pair< Sophus::SE2, Eigen::Matrix3d > ScanMatcher::run_icp_matching(const PointMatcher< double >::DataPoints& query_point_cloud, const PointMatcher< double >::DataPoints& reference_point_cloud, const Sophus::SE2 T_ref_query_init)
{
  //check point cloud dimension
  int query_dim = query_point_cloud.getEuclideanDim();
  int ref_dim = reference_point_cloud.getEuclideanDim();
  if(query_dim != ref_dim){
    ERROR_MSG("Dimension of Query and Reference Point Cloud are not equal");
  }
  if(query_dim != 2){
    ERROR_MSG("Invalid input point clouds dimension");
  }
  //Convert SE2 to Eigen::MatrixXd
  Eigen::MatrixXd T_ref_query_init_matrix(3,3);
  T_ref_query_init_matrix = T_ref_query_init.matrix();
  //Run ICP in PointMatcher
  Eigen::MatrixXd T_ref_query_matched_matrix = icp_(query_point_cloud,reference_point_cloud,T_ref_query_init_matrix);
  //Convert to 3x3
  Eigen::Matrix3d T_matched_3x3 = T_ref_query_matched_matrix;
  //Get Covariance Matrix 
  Eigen::MatrixXd cov = icp_.errorMinimizer->getCovariance();
  //Convert to 3x3
  Eigen::Matrix3d cov_3x3 = cov;
  Eigen::Matrix2d R = T_matched_3x3.block<2,2>(0,0);
  Sophus::SO2 R_ref_query_matched(R);
  Eigen::Vector2d t_ref_query_matched(T_matched_3x3.block<2,1>(0,2));
  Sophus::SE2 T_ref_query_matched(R_ref_query_matched, t_ref_query_matched);
  return std::make_pair(T_ref_query_matched,cov_3x3);
}

}