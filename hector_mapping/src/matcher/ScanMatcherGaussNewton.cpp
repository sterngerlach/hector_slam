
// ScanMatcherGaussNewton.cpp

#include "matcher/ScanMatcherGaussNewton.hpp"
#include "util/Assert.hpp"
#include "util/Parameter.hpp"
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
  OccGridMapUtilConfig<GridMap>& gridMapUtil,
  const DataContainer& dataContainer,
  Eigen::Matrix3f& covMatrix,
  const bool fineMatching)
{
  if (dataContainer.getSize() <= 0)
    return initialWorldPose;

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
      poseEstimate, dataContainer, hessianMat, residualVec);

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
  }

  // Normalize the angle from -pi to pi
  poseEstimate[2] = util::normalize_angle(poseEstimate[2]);
  // Use the Hessian matrix as the pose covariance (in the map coordinate frame)
  covMatrix = hessianMat;
  // Compute the pose estimate in the world coordinate frame
  const Eigen::Vector3f poseEstimateWorld =
    gridMapUtil.getWorldCoordsPose(poseEstimate);

  return poseEstimateWorld;
}

} // namespace hectorslam
