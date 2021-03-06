
// ScanMatcherCorrelative.hpp

#ifndef HECTOR_SLAM_MATCHER_SCAN_MATCHER_CORRELATIVE_HPP
#define HECTOR_SLAM_MATCHER_SCAN_MATCHER_CORRELATIVE_HPP

#include <memory>
#include <Eigen/Core>
#include <ros/node_handle.h>

#include "map/GridMap.h"
#include "map/GridMapCacheArray.h"
#include "map/OccGridMapUtilConfig.h"
#include "scan/DataPointContainer.h"
#include "util/DrawInterface.h"
#include "util/HectorDebugInfoInterface.h"

#include "hector_mapping/ScanMatcherCorrelativeMetrics.h"

namespace hectorslam {

//
// ScanMatcherCorrelative class implements the real-time correlative
// scan matching proposed in the following paper:
// Edwin B. Olson, "Real-Time Correlative Scan Matching," in the Proceedings of
// IEEE International Conference on Robotics and Automation (ICRA), 2009.
//
class ScanMatcherCorrelative final
{
public:
  // Load the configuration settings and create a new instance
  static std::unique_ptr<ScanMatcherCorrelative> Create(
    ros::NodeHandle& nh,
    DrawInterface* pDrawInterface = nullptr,
    HectorDebugInfoInterface* pDebugInterface = nullptr);

  // Constructor
  ScanMatcherCorrelative(const int coarseMapResolution,
                         const Eigen::Vector3f& searchWindow,
                         const float angleSearchStepMin,
                         const float angleSearchStepMax,
                         DrawInterface* pDrawInterface = nullptr,
                         HectorDebugInfoInterface* pDebugInterface = nullptr);

  // Destructor
  ~ScanMatcherCorrelative() = default;

  // Match scan to grid map given an initial world pose estimate
  Eigen::Vector3f MatchScans(
    const Eigen::Vector3f& initialWorldPose,
    const OccGridMapUtilConfig<GridMap>& gridMapUtil,
    const DataContainer& dataContainer,
    const float scoreMin,
    const float correspondenceRatioMin,
    Eigen::Matrix3f* pCovMatrix,
    hector_mapping::ScanMatcherCorrelativeMetrics* pMetrics);

private:
  // Compute the grid cell indices for scan points
  void ComputeMapIndices(
    const Eigen::Vector3f& mapPose,
    const OccGridMapUtilConfig<GridMap>& gridMapUtil,
    const DataContainer& dataContainer,
    std::vector<Eigen::Vector2i>& indices) const;

  // Compute the score using the coarse grid map
  float ComputeScoreCoarseMap(
    const OccGridMapUtilConfig<GridMap>& gridMapUtil,
    const std::vector<Eigen::Vector2i>& indices,
    const Eigen::Vector2i& offset);
  // Compute the score using the original grid map
  void ComputeScoreFineMap(
    const OccGridMapUtilConfig<GridMap>& gridMapUtil,
    const std::vector<Eigen::Vector2i>& indices,
    const Eigen::Vector3i& startEstimate,
    Eigen::Vector3i& bestEstimate, float& scoreMax);

private:
  // Resolution of a coarse grid map (in the number of grid cells)
  int mCoarseMapResolution;
  // Coarse grid map (only necessary part of it is computed)
  GridMapCacheArray mCoarseMap;
  // Search window size
  Eigen::Vector3f mSearchWindow;
  // Minimum angle search step (in radians)
  float mAngleSearchStepMin;
  // Maximum angle search step (in radians)
  float mAngleSearchStepMax;
  // Draw interface (for visualization in RViz)
  DrawInterface* mDrawInterface;
  // Debug information interface (for inspecting covariance matrices)
  HectorDebugInfoInterface* mDebugInterface;
};

} // namespace hectorslam

#endif // HECTOR_SLAM_MATCHER_SCAN_MATCHER_CORRELATIVE_HPP
