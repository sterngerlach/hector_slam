
// ScanMatcherGaussNewton.hpp

#ifndef HECTOR_SLAM_MATCHER_SCAN_MATCHER_GAUSS_NEWTON_HPP
#define HECTOR_SLAM_MATCHER_SCAN_MATCHER_GAUSS_NEWTON_HPP

#include <memory>
#include <Eigen/Core>
#include <ros/node_handle.h>

#include "map/GridMap.h"
#include "map/OccGridMapUtilConfig.h"
#include "scan/DataPointContainer.h"
#include "util/DrawInterface.h"
#include "util/HectorDebugInfoInterface.h"

namespace hectorslam {

//
// ScanMatcherGaussNewton class implements the iterative scan-to-map matching
// based on Gauss-Newton method which is described in the following paper:
// Stefan Kohlbrecher, Oskar von Stryk, Johannes Meyer, and Uwe Klingauf,
// "A Flexible and Scalable SLAM System with Full 3D Motion Estimation,"
// in the Proceedings of IEEE International Symposium on Safety, Security,
// and Rescue Robotics (SSRR), 2011.
//
class ScanMatcherGaussNewton final
{
public:
  // Load the configuration settings and create a new instance
  static std::unique_ptr<ScanMatcherGaussNewton> Create(
    ros::NodeHandle& nh,
    DrawInterface* pDrawInterface = nullptr,
    HectorDebugInfoInterface* pDebugInterface = nullptr);

  // Constructor
  ScanMatcherGaussNewton(const float angleUpdateMax,
                         const int numOfIterationsFine,
                         const int numOfIterationsCoarse,
                         DrawInterface* pDrawInterface = nullptr,
                         HectorDebugInfoInterface* pDebugInterface = nullptr);

  // Destructor
  ~ScanMatcherGaussNewton() = default;

  // Match scan to grid map given an initial world pose estimate
  Eigen::Vector3f MatchScans(
    const Eigen::Vector3f& initialWorldPose,
    OccGridMapUtilConfig<GridMap>& gridMapUtil,
    const DataContainer& dataContainer,
    Eigen::Matrix3f& covMatrix,
    const bool fineMatching);

private:
  // Maximum angle update in single Gauss-Newton iteration
  float mAngleUpdateMax;
  // Number of the iterations for fine matching
  int mNumOfIterationsFine;
  // Number of the iterations for coarse matching
  int mNumOfIterationsCoarse;
  // Draw interface (for visualization in RViz)
  DrawInterface* mDrawInterface;
  // Debug information interface (for inspecting covariance matrices)
  HectorDebugInfoInterface* mDebugInterface;
};

} // namespace hectorslam

#endif // HECTOR_SLAM_MATCHER_SCAN_MATCHER_GAUSS_NEWTON_HPP
