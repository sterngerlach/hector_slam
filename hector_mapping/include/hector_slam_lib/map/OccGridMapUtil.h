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

#ifndef HECTOR_SLAM_MAP_OCC_GRID_MAP_UTIL_H
#define HECTOR_SLAM_MAP_OCC_GRID_MAP_UTIL_H

#include <cmath>

#include "scan/DataPointContainer.h"
#include "util/UtilFunctions.h"

namespace hectorslam {

template<typename ConcreteOccGridMap, typename ConcreteCacheMethod>
class OccGridMapUtil
{
public:
  OccGridMapUtil(const ConcreteOccGridMap* gridMap) :
    concreteGridMap(gridMap)
  {
    this->mapObstacleThreshold = gridMap->getObstacleThreshold();
    this->cacheMethod.setMapSize(gridMap->getMapDimensions());
  }

  ~OccGridMapUtil() { }

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  inline Eigen::Vector3f getWorldCoordsPose(
    const Eigen::Vector3f& mapPose) const
  { return this->concreteGridMap->getWorldCoordsPose(mapPose); }
  inline Eigen::Vector3f getMapCoordsPose(
    const Eigen::Vector3f& worldPose) const
  { return this->concreteGridMap->getMapCoordsPose(worldPose); }

  inline Eigen::Vector2f getWorldCoordsPoint(
    const Eigen::Vector2f& mapPoint) const
  { return this->concreteGridMap->getWorldCoords(mapPoint); }

  inline int getMapSizeX() const
  { return this->concreteGridMap->getSizeX(); }
  inline int getMapSizeY() const
  { return this->concreteGridMap->getSizeY(); }
  inline float getScaleToMap() const
  { return this->concreteGridMap->getScaleToMap(); }
  inline float getCellLength() const
  { return this->concreteGridMap->getCellLength(); }

  inline Eigen::AlignedBox2i getBoundingBox() const
  { return this->concreteGridMap->getBoundingBox(); }

  void getCompleteHessianDerivs(const Eigen::Vector3f& pose,
                                const DataContainer& dataPoints,
                                Eigen::Matrix3f& H,
                                Eigen::Vector3f& dTr,
                                float* correspondenceCost = nullptr) const
  {
    float cost = 0.0f;

    const int size = dataPoints.getSize();

    Eigen::Affine2f transform = this->getTransformForState(pose);

    const float sinRot = std::sin(pose[2]);
    const float cosRot = std::cos(pose[2]);

    H = Eigen::Matrix3f::Zero();
    dTr = Eigen::Vector3f::Zero();

    for (int i = 0; i < size; ++i) {
      const Eigen::Vector2f& currPoint = dataPoints.getVecEntry(i);

      const Eigen::Vector3f transformedPointData =
        interpMapValueWithDerivatives(transform * currPoint);

      const float funVal = 1.0f - transformedPointData[0];
      cost += funVal;

      dTr[0] += transformedPointData[1] * funVal;
      dTr[1] += transformedPointData[2] * funVal;

      const float rotDeriv =
        (-sinRot * currPoint.x() - cosRot * currPoint.y())
        * transformedPointData[1]
        + (cosRot * currPoint.x() - sinRot * currPoint.y())
        * transformedPointData[2];

      dTr[2] += rotDeriv * funVal;

      H(0, 0) += util::sqr(transformedPointData[1]);
      H(1, 1) += util::sqr(transformedPointData[2]);
      H(2, 2) += util::sqr(rotDeriv);

      H(0, 1) += transformedPointData[1] * transformedPointData[2];
      H(0, 2) += transformedPointData[1] * rotDeriv;
      H(1, 2) += transformedPointData[2] * rotDeriv;
    }

    H(1, 0) = H(0, 1);
    H(2, 0) = H(0, 2);
    H(2, 1) = H(1, 2);

    if (correspondenceCost != nullptr)
      *correspondenceCost = cost / size;
  }

  Eigen::Matrix3f getCovarianceForPose(const Eigen::Vector3f& mapPose,
                                       const DataContainer& dataPoints) const
  {
    float deltaTransX = 1.5f;
    float deltaTransY = 1.5f;
    float deltaAng = 0.05f;

    float x = mapPose[0];
    float y = mapPose[1];
    float ang = mapPose[2];

    Eigen::Matrix<float, 3, 7> sigmaPoints;

    sigmaPoints.block<3, 1>(0, 0) = Eigen::Vector3f(x + deltaTransX, y, ang);
    sigmaPoints.block<3, 1>(0, 1) = Eigen::Vector3f(x - deltaTransX, y, ang);
    sigmaPoints.block<3, 1>(0, 2) = Eigen::Vector3f(x, y + deltaTransY, ang);
    sigmaPoints.block<3, 1>(0, 3) = Eigen::Vector3f(x, y - deltaTransY, ang);
    sigmaPoints.block<3, 1>(0, 4) = Eigen::Vector3f(x, y, ang + deltaAng);
    sigmaPoints.block<3, 1>(0, 5) = Eigen::Vector3f(x, y, ang - deltaAng);
    sigmaPoints.block<3, 1>(0, 6) = mapPose;

    Eigen::Matrix<float, 7, 1> likelihoods;

    likelihoods[0] = this->getLikelihoodForState(
      Eigen::Vector3f(x + deltaTransX, y, ang), dataPoints);
    likelihoods[1] = this->getLikelihoodForState(
      Eigen::Vector3f(x - deltaTransX, y, ang), dataPoints);
    likelihoods[2] = this->getLikelihoodForState(
      Eigen::Vector3f(x, y + deltaTransY, ang), dataPoints);
    likelihoods[3] = this->getLikelihoodForState(
      Eigen::Vector3f(x, y - deltaTransY, ang), dataPoints);
    likelihoods[4] = this->getLikelihoodForState(
      Eigen::Vector3f(x, y, ang + deltaAng), dataPoints);
    likelihoods[5] = this->getLikelihoodForState(
      Eigen::Vector3f(x, y, ang - deltaAng), dataPoints);
    likelihoods[6] = this->getLikelihoodForState(
      Eigen::Vector3f(x, y, ang), dataPoints);

    float invLhNormalizer = 1.0f / likelihoods.sum();

    Eigen::Vector3f mean = Eigen::Vector3f::Zero();

    for (int i = 0; i < 7; ++i)
      mean += sigmaPoints.block<3, 1>(0, i) * likelihoods[i];

    mean *= invLhNormalizer;

    Eigen::Matrix3f covMatrixMap = Eigen::Matrix3f::Zero();

    for (int i = 0; i < 7; ++i) {
      Eigen::Vector3f sigPointMinusMean = sigmaPoints.block<3, 1>(0, i) - mean;
      covMatrixMap += (likelihoods[i] * invLhNormalizer)
        * (sigPointMinusMean * sigPointMinusMean.transpose());
    }

    return covMatrixMap;
  }

  Eigen::Matrix3f getCovMatrixWorldCoords(
    const Eigen::Matrix3f& covMatMap) const
  {
    Eigen::Matrix3f covMatWorld;

    float scaleTrans = this->concreteGridMap->getCellLength();
    float scaleTransSq = util::sqr(scaleTrans);

    covMatWorld(0, 0) = covMatMap(0, 0) * scaleTransSq;
    covMatWorld(1, 1) = covMatMap(1, 1) * scaleTransSq;

    covMatWorld(1, 0) = covMatMap(1, 0) * scaleTransSq;
    covMatWorld(0, 1) = covMatWorld(1, 0);

    covMatWorld(2, 0) = covMatMap(2, 0) * scaleTrans;
    covMatWorld(0, 2) = covMatWorld(2, 0);

    covMatWorld(2, 1) = covMatMap(2, 1) * scaleTrans;
    covMatWorld(1, 2) = covMatWorld(2, 1);

    covMatWorld(2, 2) = covMatMap(2, 2);

    return covMatWorld;
  }

  float getLikelihoodForState(const Eigen::Vector3f& state,
                              const DataContainer& dataPoints) const
  {
    float resid = this->getResidualForState(state, dataPoints);
    return this->getLikelihoodForResidual(resid, dataPoints.getSize());
  }

  float getLikelihoodForResidual(float residual, int numDataPoints) const
  { return 1.0f - (residual / static_cast<float>(numDataPoints)); }

  float getResidualForState(const Eigen::Vector3f& state,
                            const DataContainer& dataPoints) const
  {
    const int size = dataPoints.getSize();

    int stepSize = 1;
    float residual = 0.0f;

    const Eigen::Affine2f transform = this->getTransformForState(state);

    for (int i = 0; i < size; i += stepSize) {
      const float funval = 1.0f - this->interpMapValue(
        transform * dataPoints.getVecEntry(i));
      residual += funval;
    }

    return residual;
  }

  inline float getUnfilteredGridPoint(Eigen::Vector2i& gridCoords) const
  { return this->concreteGridMap->getGridProbabilityMap(
    gridCoords.x(), gridCoords.y()); }

  inline float getUnfilteredGridPoint(int index) const
  { return this->concreteGridMap->getGridProbabilityMap(index); }

  float interpMapValue(const Eigen::Vector2f& coords) const
  {
    // Check if coords are within map limits.
    if (this->concreteGridMap->pointOutOfMapBounds(coords))
      return 0.0f;

    Eigen::Vector4f intensities;

    // Map coords are alway positive, floor them by casting to int
    const Eigen::Vector2i indMin = coords.cast<int>();
    // Get factors for bilinear interpolation
    const Eigen::Vector2f factors = coords - indMin.cast<float>();

    const int sizeX = this->concreteGridMap->getSizeX();

    int index = indMin[1] * sizeX + indMin[0];

    // Get grid values for the 4 grid points surrounding the current coords.
    // Check cached data first, if not contained filter gridPoint
    // with gaussian and store in cache.
    if (!this->cacheMethod.containsCachedData(index, intensities[0])) {
      intensities[0] = this->getUnfilteredGridPoint(index);
      this->cacheMethod.cacheData(index, intensities[0]);
    }

    ++index;

    if (!this->cacheMethod.containsCachedData(index, intensities[1])) {
      intensities[1] = this->getUnfilteredGridPoint(index);
      this->cacheMethod.cacheData(index, intensities[1]);
    }

    index += sizeX - 1;

    if (!this->cacheMethod.containsCachedData(index, intensities[2])) {
      intensities[2] = this->getUnfilteredGridPoint(index);
      this->cacheMethod.cacheData(index, intensities[2]);
    }

    ++index;

    if (!this->cacheMethod.containsCachedData(index, intensities[3])) {
      intensities[3] = this->getUnfilteredGridPoint(index);
      this->cacheMethod.cacheData(index, intensities[3]);
    }

    const float xFacInv = 1.0f - factors[0];
    const float yFacInv = 1.0f - factors[1];

    return ((intensities[0] * xFacInv + intensities[1] * factors[0]) * yFacInv)
      + ((intensities[2] * xFacInv + intensities[3] * factors[0]) * factors[1]);
  }

  Eigen::Vector3f interpMapValueWithDerivatives(
    const Eigen::Vector2f& coords) const
  {
    // Check if coords are within map limits.
    if (this->concreteGridMap->pointOutOfMapBounds(coords))
      return Eigen::Vector3f::Zero();

    Eigen::Vector4f intensities;

    // Map coords are always positive, floor them by casting to int
    const Eigen::Vector2i indMin = coords.cast<int>();
    // Get factors for bilinear interpolation
    const Eigen::Vector2f factors = coords - indMin.cast<float>();

    const int sizeX = this->concreteGridMap->getSizeX();

    int index = indMin[1] * sizeX + indMin[0];

    // Get grid values for the 4 grid points surrounding the current coords.
    // Check cached data first, if not contained filter gridPoint
    // with gaussian and store in cache.
    if (!this->cacheMethod.containsCachedData(index, intensities[0])) {
      intensities[0] = this->getUnfilteredGridPoint(index);
      this->cacheMethod.cacheData(index, intensities[0]);
    }

    ++index;

    if (!this->cacheMethod.containsCachedData(index, intensities[1])) {
      intensities[1] = this->getUnfilteredGridPoint(index);
      this->cacheMethod.cacheData(index, intensities[1]);
    }

    index += sizeX - 1;

    if (!this->cacheMethod.containsCachedData(index, intensities[2])) {
      intensities[2] = this->getUnfilteredGridPoint(index);
      this->cacheMethod.cacheData(index, intensities[2]);
    }

    ++index;

    if (!this->cacheMethod.containsCachedData(index, intensities[3])) {
      intensities[3] = this->getUnfilteredGridPoint(index);
      this->cacheMethod.cacheData(index, intensities[3]);
    }

    // `intensities` represent the occupancy probability values at the
    // grid cells (x, y), (x + 1, y), (x, y + 1), (x + 1, y + 1)
    const float dx1 = intensities[0] - intensities[1];
    const float dx2 = intensities[2] - intensities[3];

    const float dy1 = intensities[0] - intensities[2];
    const float dy2 = intensities[1] - intensities[3];

    const float xFacInv = 1.0f - factors[0];
    const float yFacInv = 1.0f - factors[1];

    return Eigen::Vector3f {
      ((intensities[0] * xFacInv + intensities[1] * factors[0]) * yFacInv)
      + ((intensities[2] * xFacInv + intensities[3] * factors[0]) * factors[1]),
      -(dx1 * yFacInv + dx2 * factors[1]),
      -(dy1 * xFacInv + dy2 * factors[0]) };
  }

  inline Eigen::Affine2f getTransformForState(
    const Eigen::Vector3f& transVector) const
  { return Eigen::Translation2f(transVector[0], transVector[1])
           * Eigen::Rotation2Df(transVector[2]); }

  inline Eigen::Translation2f getTranslationForState(
    const Eigen::Vector3f& transVector) const
  { return Eigen::Translation2f(transVector[0], transVector[1]); }

  inline void resetCachedData() { this->cacheMethod.resetCache(); }

protected:
  mutable ConcreteCacheMethod cacheMethod;
  const ConcreteOccGridMap* concreteGridMap;
  float mapObstacleThreshold;
};

} // namespace hectorslam

#endif
