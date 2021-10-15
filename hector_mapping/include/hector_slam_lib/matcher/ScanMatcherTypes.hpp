
// ScanMatcherTypes.hpp

#ifndef HECTOR_SLAM_MATCHER_SCAN_MATCHER_TYPES_HPP
#define HECTOR_SLAM_MATCHER_SCAN_MATCHER_TYPES_HPP

#include <functional>
#include <Eigen/Core>

#include "map/GridMap.h"
#include "matcher/ScanMatcher.h"
#include "matcher/ScanMatcherCorrelative.hpp"
#include "matcher/ScanMatcherFPGA.hpp"
#include "matcher/ScanMatcherGaussNewton.hpp"
#include "scan/DataPointContainer.h"

namespace hectorslam {

// Type definition for the default scan matcher
using DefaultScanMatcher = ScanMatcher<GridMapUtil>;

// Type definition for the scan matcher callback which takes the initial
// pose estimate, occupancy grid map, scan, reference to the resulting
// pose covariance, and index of the map, and returns the pose estimate
using ScanMatchCallback = std::function<
  Eigen::Vector3f(const Eigen::Vector3f&, GridMapUtil&,
    const DataContainer&, Eigen::Matrix3f&, const int)>;

} // namespace hectorslam

#endif // HECTOR_SLAM_MATCHER_SCAN_MATCHER_TYPES_HPP
