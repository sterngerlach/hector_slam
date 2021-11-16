//=================================================================================================
// Copyright (c) 2011, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef HECTOR_SLAM_MATCHER_SCAN_MATCHER_H
#define HECTOR_SLAM_MATCHER_SCAN_MATCHER_H

#include <memory>
#include <Eigen/Geometry>
#include <ros/node_handle.h>

#include "scan/DataPointContainer.h"
#include "util/DrawInterface.h"
#include "util/HectorDebugInfoInterface.h"
#include "util/UtilFunctions.h"

namespace hectorslam {

template<typename ConcreteOccGridMapUtil>
class ScanMatcher
{
public:
  ScanMatcher(DrawInterface* drawInterfaceIn = nullptr,
              HectorDebugInfoInterface* debugInterfaceIn = nullptr) :
    drawInterface(drawInterfaceIn),
    debugInterface(debugInterfaceIn) { }

  ~ScanMatcher() { }

  // Load the configuration settings and create a new instance
  static std::unique_ptr<ScanMatcher> Create(
    ros::NodeHandle& nh,
    DrawInterface* pDrawInterface = nullptr,
    HectorDebugInfoInterface* pDebugInterface = nullptr)
  {
    return std::make_unique<ScanMatcher>(pDrawInterface, pDebugInterface);
  }

  Eigen::Vector3f matchData(const Eigen::Vector3f& beginEstimateWorld,
                            ConcreteOccGridMapUtil& gridMapUtil,
                            const DataContainer& dataContainer,
                            Eigen::Matrix3f& covMatrix, int maxIterations)
  {
    if (this->drawInterface != nullptr) {
      this->drawInterface->setScale(0.05f);
      this->drawInterface->setColor(0.0f, 1.0f, 0.0f);
      this->drawInterface->drawArrow(beginEstimateWorld);

      const Eigen::Vector3f beginEstimateMap =
        gridMapUtil.getMapCoordsPose(beginEstimateWorld);

      this->drawScan(beginEstimateMap, gridMapUtil, dataContainer);
      this->drawInterface->setColor(1.0, 0.0, 0.0);
    }

    if (dataContainer.getSize() <= 0)
      return beginEstimateWorld;

    Eigen::Vector3f beginEstimateMap =
      gridMapUtil.getMapCoordsPose(beginEstimateWorld);
    Eigen::Vector3f estimate = beginEstimateMap;

    this->estimateTransformationLogLh(estimate, gridMapUtil, dataContainer);
    // bool notConverged = this->estimateTransformationLogLh(
    //   estimate, gridMapUtil, dataContainer);

    int numIter = maxIterations;

    for (int i = 0; i < numIter; ++i) {
      this->estimateTransformationLogLh(estimate, gridMapUtil, dataContainer);
      // notConverged = this->estimateTransformationLogLh(
      //   estimate, gridMapUtil, dataContainer);

      if (this->drawInterface != nullptr) {
        const float invNumIterf = 1.0f / static_cast<float>(numIter);
        this->drawInterface->setColor(
          static_cast<float>(i) * invNumIterf, 0.0f, 0.0f);
        this->drawInterface->drawArrow(
          gridMapUtil.getWorldCoordsPose(estimate));
        // this->drawInterface->drawArrow(
        //   Eigen::Vector3f(0.0f, static_cast<float>(i) * 0.05, 0.0f));
      }

      if (this->debugInterface != nullptr)
        this->debugInterface->addHessianMatrix(H);
    }

    if (this->drawInterface != nullptr) {
      this->drawInterface->setColor(0.0, 0.0, 1.0);
      this->drawScan(estimate, gridMapUtil, dataContainer);
    }

    /*
    Eigen::Matrix2f testMat = Eigen::Matrix2f::Identity();
    testMat(0, 0) = 2.0f;

    float angleWorldCoords = util::toRad(30.0f);
    float sinAngle = std::sin(angleWorldCoords);
    float cosAngle = std::cos(angleWorldCoords);

    Eigen::Matrix2f rotMat;
    rotMat << cosAngle, -sinAngle, sinAngle, cosAngle;
    Eigen::Matrix2f covarianceRotated = rotMat * testMat * rotMat.transpose();

    this->drawInterface->setColor(0.0, 0.0, 1.0, 0.5);
    this->drawInterface->drawCovariance(gridMapUtil.getWorldCoordsPoint(
      estimate.start<2>()), covarianceRotated);
    */

    /*
    Eigen::Matrix3f covMatMap =
      gridMapUtil.getCovarianceForPose(estimate, dataContainer);
    this->drawInterface->setColor(0.0, 0.0, 1.0, 0.5);

    Eigen::Matrix3f covMatWorld =
      gridMapUtil.getCovMatrixWorldCoords(covMatMap);

    this->drawInterface->drawCovariance(gridMapUtil.getWorldCoordsPoint(
      estimate.start<2>()), covMatMap.block<2, 2>(0, 0));

    this->drawInterface->setColor(1.0, 0.0, 0.0, 0.5);
    this->drawInterface->drawCovariance(gridMapUtil.getWorldCoordsPoint(
      estimate.start<2>()), covMatWorld.block<2, 2>(0, 0));

    float determinant = H.determinant();
    */

    /*
    Eigen::Matrix2f covFromHessian = H.block<2, 2>(0, 0) * 1.0f;

    this->drawInterface->setColor(0.0, 1.0, 0.0, 0.5);
    this->drawInterface->drawCovariance(gridMapUtil.getWorldCoordsPoint(
      estimate.start<2>()),covFromHessian.inverse());

    Eigen::Matrix3f covFromHessian3d(H * 1.0f);

    this->drawInterface->setColor(1.0, 0.0, 0.0, 0.8);
    this->drawInterface->drawCovariance(gridMapUtil.getWorldCoordsPoint(
      estimate.start<2>()), (covFromHessian3d.inverse()).block<2, 2>(0, 0));
    */

    // covMatrix = Eigen::Matrix3f::Zero();
    // covMatrix.block<2, 2>(0, 0) = H.block<2, 2>(0, 0).inverse();
    // covMatrix.block<2, 2>(0, 0) = H.block<2, 2>(0, 0);

    // covMatrix(0, 0) = 1.0 / (0.1 * 0.1);
    // covMatrix(1, 1) = 1.0 / (0.1 * 0.1);
    // covMatrix(2, 2) = 1.0 / ((M_PI / 18.0f) * (M_PI / 18.0f));

    estimate[2] = util::normalize_angle(estimate[2]);
    covMatrix = H;

    return gridMapUtil.getWorldCoordsPose(estimate);
  }

protected:
  bool estimateTransformationLogLh(Eigen::Vector3f& estimate,
                                   ConcreteOccGridMapUtil& gridMapUtil,
                                   const DataContainer& dataPoints)
  {
    gridMapUtil.getCompleteHessianDerivs(estimate, dataPoints, H, dTr);

    if (H(0, 0) != 0.0f && H(1, 1) != 0.0f) {
      // H += Eigen::Matrix3f::Identity() * 1.0f;
      Eigen::Vector3f searchDir = H.inverse() * dTr;

      if (searchDir[2] > 0.2f) {
        searchDir[2] = 0.2f;
        std::cout << "SearchDir angle change too large\n";
      } else if (searchDir[2] < -0.2f) {
        searchDir[2] = -0.2f;
        std::cout << "SearchDir angle change too large\n";
      }

      this->updateEstimatedPose(estimate, searchDir);
      return true;
    }

    return false;
  }

  void updateEstimatedPose(Eigen::Vector3f& estimate,
                           const Eigen::Vector3f& change)
  {
    estimate += change;
  }

  void drawScan(const Eigen::Vector3f& pose,
                const ConcreteOccGridMapUtil& gridMapUtil,
                const DataContainer& dataContainer)
  {
    this->drawInterface->setScale(0.02);

    Eigen::Affine2f transform = gridMapUtil.getTransformForState(pose);

    int size = dataContainer.getSize();
    for (int i = 0; i < size; ++i) {
      const Eigen::Vector2f& currPoint = dataContainer.getVecEntry(i);
      this->drawInterface->drawPoint(gridMapUtil.getWorldCoordsPoint(
        transform * currPoint));
    }
  }

protected:
  Eigen::Vector3f dTr;
  Eigen::Matrix3f H;

  DrawInterface* drawInterface;
  HectorDebugInfoInterface* debugInterface;
};

} // namespace hectorslam

#endif // HECTOR_SLAM_MATCHER_SCAN_MATCHER_H
