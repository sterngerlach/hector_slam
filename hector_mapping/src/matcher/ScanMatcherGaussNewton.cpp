
// ScanMatcherGaussNewton.cpp

#include "matcher/ScanMatcherGaussNewton.hpp"
#include "util/Assert.hpp"
#include "util/UtilFunctions.h"

namespace hectorslam {

// Constructor
ScanMatcherGaussNewton::ScanMatcherGaussNewton(
  const float angleUpdateMax,
  DrawInterface* pDrawInterface,
  HectorDebugInfoInterface* pDebugInterface) :
  mAngleUpdateMax(angleUpdateMax),
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
  const int maxIterations)
{
  if (dataContainer.getSize() <= 0)
    return initialWorldPose;

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
