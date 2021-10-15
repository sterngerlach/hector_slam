
// ScanMatcherOptions.hpp

#ifndef HECTOR_SLAM_MATCHER_SCAN_MATCHER_OPTIONS_HPP
#define HECTOR_SLAM_MATCHER_SCAN_MATCHER_OPTIONS_HPP

namespace hectorslam {

//
// ScanMatcherOptions enum specifies which scan matchers to use in Hector SLAM
// Note that correlative scan matcher, Gauss-Newton scan matcher, and
// FPGA-based correlative scan matcher are the possible choices
//
enum class ScanMatcherOptions
{
  Default,
  GaussNewton,
  Correlative,
  CorrelativeFPGA,
  GaussNewtonAfterCorrelative,
  GaussNewtonAfterCorrelativeFPGA
};

} // namespace hectorslam

#endif // HECTOR_SLAM_MATCHER_SCAN_MATCHER_OPTIONS_HPP
