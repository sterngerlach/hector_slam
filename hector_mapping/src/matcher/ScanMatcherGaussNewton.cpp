
// ScanMatcherGaussNewton.cpp

#include "matcher/ScanMatcherGaussNewton.hpp"
#include "util/Assert.hpp"
#include "util/Parameter.hpp"
#include "util/Timer.hpp"
#include "util/UtilFunctions.h"

#define RETURN_NULLPTR_IF_FAILED(call) if (!(call)) return nullptr;

namespace hectorslam {

// Load the configuration settings and create a new instance
std::unique_ptr<ScanMatcherGaussNewton> ScanMatcherGaussNewton::Create(
  ros::NodeHandle& nh,
  DrawInterface* pDrawInterface,
  HectorDebugInfoInterface* pDebugInterface)
{
  ros::NodeHandle nhScanMatcher { nh, "scan_matcher_gauss_newton" };

  float angleUpdateMax;
  int numOfIterationsFine;
  int numOfIterationsCoarse;

  RETURN_NULLPTR_IF_FAILED(GetParamFromNodeHandle(
    nhScanMatcher, "angle_update_max", angleUpdateMax));
  RETURN_NULLPTR_IF_FAILED(GetParamFromNodeHandle(
    nhScanMatcher, "num_of_iterations_fine", numOfIterationsFine))
  RETURN_NULLPTR_IF_FAILED(GetParamFromNodeHandle(
    nhScanMatcher, "num_of_iterations_coarse", numOfIterationsCoarse))

  return std::make_unique<ScanMatcherGaussNewton>(
    angleUpdateMax, numOfIterationsFine, numOfIterationsCoarse,
    pDrawInterface, pDebugInterface);
}

// Constructor
ScanMatcherGaussNewton::ScanMatcherGaussNewton(
  const float angleUpdateMax,
  const int numOfIterationsFine,
  const int numOfIterationsCoarse,
  DrawInterface* pDrawInterface,
  HectorDebugInfoInterface* pDebugInterface) :
  mAngleUpdateMax(angleUpdateMax),
  mNumOfIterationsFine(numOfIterationsFine),
  mNumOfIterationsCoarse(numOfIterationsCoarse),
  mDrawInterface(pDrawInterface),
  mDebugInterface(pDebugInterface)
{
}

// Match scan to grid map given an initial world pose estimate
Eigen::Vector3f ScanMatcherGaussNewton::MatchScans(
  const Eigen::Vector3f& initialWorldPose,
  const OccGridMapUtilConfig<GridMap>& gridMapUtil,
  const DataContainer& dataContainer,
  const bool fineMatching,
  Eigen::Matrix3f* pCovMatrix,
  hector_mapping::ScanMatcherGaussNewtonMetrics* pMetrics)
{
  if (dataContainer.getSize() <= 0)
    return initialWorldPose;

  Timer timer;
  int numOfIterations = 0;
  float correspondenceCost = 0.0f;
  float initialCorrespondenceCost = 0.0f;

  // Number of the iterations
  const int maxIterations = fineMatching ?
    this->mNumOfIterationsFine : this->mNumOfIterationsCoarse;
  // Compute the pose in the map coordinate frame
  const Eigen::Vector3f initialMapPose =
    gridMapUtil.getMapCoordsPose(initialWorldPose);
  // Initialize the pose estimate
  Eigen::Vector3f poseEstimate = initialMapPose;
  // Initialize the Hessian matrix and residual vector
  Eigen::Matrix3f hessianMat;
  Eigen::Vector3f residualVec;

  for (int i = 0; i < maxIterations; ++i) {
    // Compute the Hessian matrix and residual vector
    gridMapUtil.getCompleteHessianDerivs(
      poseEstimate, dataContainer,
      hessianMat, residualVec, &correspondenceCost);

    if (i == 0)
      initialCorrespondenceCost = correspondenceCost;

    if (hessianMat(0, 0) == 0.0f || hessianMat(1, 1) == 0.0f)
      break;

    Eigen::Vector3f poseUpdate = hessianMat.inverse() * residualVec;

    // Clip the angle update
    if (poseUpdate[2] > this->mAngleUpdateMax)
      poseUpdate[2] = this->mAngleUpdateMax;
    else if (poseUpdate[2] < -this->mAngleUpdateMax)
      poseUpdate[2] = -this->mAngleUpdateMax;

    // Update the pose estimate
    poseEstimate += poseUpdate;
    // Update the number of iterations
    numOfIterations++;
  }

  // Normalize the angle from -pi to pi
  poseEstimate[2] = util::normalize_angle(poseEstimate[2]);
  // Compute the pose estimate in the world coordinate frame
  const Eigen::Vector3f poseEstimateWorld =
    gridMapUtil.getWorldCoordsPose(poseEstimate);

  // Use the Hessian matrix as the pose covariance (in the map coordinate frame)
  if (pCovMatrix != nullptr)
    *pCovMatrix = hessianMat;

  // Fill the metrics information if necessary
  if (pMetrics != nullptr) {
    const Eigen::Vector3f poseDiffWorld = poseEstimateWorld - initialWorldPose;
    pMetrics->optimization_time.fromNSec(timer.ElapsedNanoseconds());
    pMetrics->diff_translation = poseDiffWorld.head<2>().norm();
    pMetrics->diff_rotation = util::NormalizeAngleDifference(poseDiffWorld[2]);
    pMetrics->num_of_iterations = numOfIterations;
    pMetrics->initial_correspondence_cost = initialCorrespondenceCost;
    pMetrics->final_correspondence_cost = correspondenceCost;
    pMetrics->num_of_scans = dataContainer.getSize();
  }

  return poseEstimateWorld;
}

} // namespace hectorslam
