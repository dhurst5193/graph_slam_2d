#ifndef SCAN_MATCHER_H_
#define SCAN_MATCHER_H_

#include "graph_slam_2d/common.hh"
#include "pointmatcher/IO.h"
#include "pointmatcher/PointMatcher.h"

namespace graph_slam_2d{
  class ScanMatcher{
  public:
    using Ptr = std::shared_ptr<ScanMatcher>;
    using MatcherType = PointMatcher<double>;
    
    ScanMatcher();
    /*!
     * @brief Run ICP Matching using PointMatcher ICP program
     * @param query_point_cloud Query point cloud to be match 
     * @param reference_point_cloud Reference point cloud to match to
     * @param T_ref_query_init Initial Transformation from query frame to reference frame
     * @return Matched relative transformation from query to reference point cloud frame 
     */
    std::pair<Sophus::SE2,Eigen::Matrix3d> run_icp_matching(const MatcherType::DataPoints& query_point_cloud, 
				 const MatcherType::DataPoints& reference_point_cloud,
				 const Sophus::SE2 T_ref_query_init
				);
  private:
    MatcherType::ICP icp_;
    
  };
}

#endif