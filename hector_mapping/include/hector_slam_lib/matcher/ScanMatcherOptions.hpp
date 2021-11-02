
// ScanMatcherOptions.hpp

#ifndef HECTOR_SLAM_MATCHER_SCAN_MATCHER_OPTIONS_HPP
#define HECTOR_SLAM_MATCHER_SCAN_MATCHER_OPTIONS_HPP

#include <string>

namespace hectorslam {

//
// ScanMatcherOption enum specifies which scan matchers to use in Hector SLAM
// Note that correlative scan matcher, Gauss-Newton scan matcher, and
// FPGA-based correlative scan matcher are the possible choices
//
enum class ScanMatcherOption
{
  Unknown,
  Default,
  GaussNewton,
  Correlative,
  CorrelativeFPGA,
  GaussNewtonAfterCorrelative,
  GaussNewtonAfterCorrelativeFPGA
};

// Convert the given string to ScanMatcherOption enum
inline ScanMatcherOption StringToScanMatcherOption(const std::string& value)
{
  if (value == "Default")
    return ScanMatcherOption::Default;
  if (value == "GaussNewton")
    return ScanMatcherOption::GaussNewton;
  if (value == "Correlative")
    return ScanMatcherOption::Correlative;
  if (value == "CorrelativeFPGA")
    return ScanMatcherOption::CorrelativeFPGA;
  if (value == "GaussNewtonAfterCorrelative")
    return ScanMatcherOption::GaussNewtonAfterCorrelative;
  if (value == "GaussNewtonAfterCorrelativeFPGA")
    return ScanMatcherOption::GaussNewtonAfterCorrelativeFPGA;

  return ScanMatcherOption::Unknown;
}

// Convert the given ScanMatcherOption enum to string
inline const char* ScanMatcherOptionToString(const ScanMatcherOption option)
{
  switch (option) {
    case ScanMatcherOption::Unknown:
      return "Unknown";
    case ScanMatcherOption::GaussNewton:
      return "GaussNewton";
    case ScanMatcherOption::Correlative:
      return "Correlative";
    case ScanMatcherOption::CorrelativeFPGA:
      return "CorrelativeFPGA";
    case ScanMatcherOption::GaussNewtonAfterCorrelative:
      return "GaussNewtonAfterCorrelative";
    case ScanMatcherOption::GaussNewtonAfterCorrelativeFPGA:
      return "GaussNewtonAfterCorrelativeFPGA";
  }

  return "";
}

} // namespace hectorslam

#endif // HECTOR_SLAM_MATCHER_SCAN_MATCHER_OPTIONS_HPP
