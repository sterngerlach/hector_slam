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

#ifndef HECTOR_SLAM_MAP_OCC_GRID_MAP_BASE_H
#define HECTOR_SLAM_MAP_OCC_GRID_MAP_BASE_H

#include <limits>
#include <type_traits>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "map/GridMapBase.h"
#include "map/GridMapLogOdds.h"
#include "map/GridMapSimpleCount.h"
#include "scan/DataPointContainer.h"
#include "util/UtilFunctions.h"

namespace hectorslam {

template <typename ConcreteCellType, typename ConcreteGridFunctions>
class OccGridMapBase : public GridMapBase<ConcreteCellType>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  OccGridMapBase(const float mapResolution,
                 const Eigen::Vector2i& size,
                 const Eigen::Vector2f& offset) :
    GridMapBase<ConcreteCellType>(mapResolution, size, offset),
    currUpdateIndex(0),
    currMarkOccIndex(-1),
    currMarkFreeIndex(-1) { }

  virtual ~OccGridMapBase() { }

  inline void updateSetOccupied(int index)
  { this->concreteGridFunctions.updateSetOccupied(this->getCell(index)); }
  inline void updateSetFree(int index)
  { this->concreteGridFunctions.updateSetFree(this->getCell(index)); }
  inline void updateUnsetFree(int index)
  { this->concreteGridFunctions.updateUnsetFree(this->getCell(index)); }

  inline float getGridProbabilityMap(int index) const
  { return this->concreteGridFunctions.getGridProbability(
    this->getCell(index)); }

  inline bool isOccupied(int xMap, int yMap) const
  { return this->getCell(xMap,yMap).isOccupied(); }
  inline bool isFree(int xMap, int yMap) const
  { return this->getCell(xMap,yMap).isFree(); }

  inline bool isOccupied(int index) const
  { return this->getCell(index).isOccupied(); }
  inline bool isFree(int index) const
  { return this->getCell(index).isFree(); }

  inline float getObstacleThreshold() const
  {
    ConcreteCellType temp;
    temp.resetGridCell();
    return this->concreteGridFunctions.getGridProbability(temp);
  }

  inline void setUpdateFreeFactor(float factor)
  { this->concreteGridFunctions.setUpdateFreeFactor(factor); }
  inline void setUpdateOccupiedFactor(float factor)
  { this->concreteGridFunctions.setUpdateOccupiedFactor(factor); }

  /**
   * Updates the map using the given scan data and robot pose
   * @param dataContainer Contains the laser scan data
   * @param robotPoseWorld The 2D robot pose in world coordinates
   */
  void updateByScan(const DataContainer& dataContainer,
                    const Eigen::Vector3f& robotPoseWorld)
  {
    this->currMarkFreeIndex = this->currUpdateIndex + 1;
    this->currMarkOccIndex = this->currUpdateIndex + 2;

    // Get pose in map coordinates from pose in world coordinates
    const Eigen::Vector3f mapPose = this->getMapCoordsPose(robotPoseWorld);

    // Get a 2D homogenous transform that can be left-multiplied to
    // a robot coordinates vector to get world coordinates of that vector
    const Eigen::Affine2f poseTransform =
      Eigen::Translation2f(mapPose[0], mapPose[1])
        * Eigen::Rotation2Df(mapPose[2]);

    // Get start point of all laser beams in map coordinates
    // (same for all beams, stored in robot coords in dataContainer)
    const Eigen::Vector2f scanBeginMapf =
      poseTransform * dataContainer.getOrigo();

    // Get integer vector of laser beams start point
    const Eigen::Vector2i scanBeginMapi {
      scanBeginMapf[0] + 0.5f, scanBeginMapf[1] + 0.5f };

    // Get number of valid beams in current scan
    const int numValidElems = dataContainer.getSize();

    // Iterate over all valid laser beams
    for (int i = 0; i < numValidElems; ++i) {
      // Get map coordinates of current beam endpoint
      Eigen::Vector2f scanEndMapf =
        poseTransform * dataContainer.getVecEntry(i);

      // Add 0.5 to beam endpoint vector for following integer cast
      // (to round, not truncate)
      scanEndMapf.array() += 0.5f;

      // Get integer map coordinates of current beam endpoint
      const Eigen::Vector2i scanEndMapi = scanEndMapf.cast<int>();

      // Update map using a bresenham variant for drawing a line
      // from beam start to beam endpoint in map coordinates
      if (scanBeginMapi != scanEndMapi)
        this->updateLineBresenhami(scanBeginMapi, scanEndMapi);
    }

    // Tell the map that it has been updated
    this->setUpdated();

    // Increase update index (used for updating grid cells
    // only once per incoming scan)
    this->currUpdateIndex += 3;
  }

  void updateLineBresenhami(const Eigen::Vector2i& beginMap,
                            const Eigen::Vector2i& endMap)
  {
    const int x0 = beginMap[0];
    const int y0 = beginMap[1];
    const int x1 = endMap[0];
    const int y1 = endMap[1];

    // Check if beam start point is inside map,
    // cancel update if this is not the case
    if (x0 < 0 || x0 >= this->getSizeX() || y0 < 0 || y0 >= this->getSizeY())
      return;

    // Check if beam end point is inside map,
    // cancel update if this is not the case
    if (x1 < 0 || x1 >= this->getSizeX() || y1 < 0 || y1 >= this->getSizeY())
      return;

    const int dx = x1 - x0;
    const int dy = y1 - y0;
    const unsigned int abs_dx = std::abs(dx);
    const unsigned int abs_dy = std::abs(dy);
    const int offset_dx = util::sign(dx);
    const int offset_dy = util::sign(dy) * this->sizeX;

    const unsigned int startOffset = beginMap.y() * this->sizeX + beginMap.x();
    const unsigned int endOffset = endMap.y() * this->sizeX + endMap.x();

    // If x is dominant
    if (abs_dx >= abs_dy) {
      int error_y = abs_dx / 2;
      this->bresenham2D(abs_dx, abs_dy, error_y,
                        offset_dx, offset_dy, startOffset);
    } else {
      // Otherwise y is dominant
      int error_x = abs_dy / 2;
      this->bresenham2D(abs_dy, abs_dx, error_x,
                        offset_dy, offset_dx, startOffset);
    }

    this->bresenhamCellOcc(endOffset);
  }

  inline void bresenhamCellFree(unsigned int offset)
  {
    ConcreteCellType& cell = this->getCell(offset);

    if (cell.updateIndex < this->currMarkFreeIndex) {
      this->concreteGridFunctions.updateSetFree(cell);
      cell.updateIndex = this->currMarkFreeIndex;
    }
  }

  inline void bresenhamCellOcc(unsigned int offset)
  {
    ConcreteCellType& cell = this->getCell(offset);

    if (cell.updateIndex < this->currMarkOccIndex) {
      // If this cell has been updated as free in the current iteration,
      // revert this
      if (cell.updateIndex == this->currMarkFreeIndex)
        this->concreteGridFunctions.updateUnsetFree(cell);

      this->concreteGridFunctions.updateSetOccupied(cell);
      cell.updateIndex = this->currMarkOccIndex;
    }
  }

  inline void bresenham2D(unsigned int abs_da, unsigned int abs_db,
                          int error_b, int offset_a, int offset_b,
                          unsigned int offset)
  {
    this->bresenhamCellFree(offset);

    const unsigned int end = abs_da - 1;

    for (unsigned int i = 0; i < end; ++i) {
      offset += offset_a;
      error_b += abs_db;

      if (static_cast<unsigned int>(error_b) >= abs_da) {
        offset += offset_b;
        error_b -= abs_da;
      }

      this->bresenhamCellFree(offset);
    }
  }

  template <typename std::enable_if_t<std::is_same<
    ConcreteCellType, LogOddsCell>::value, std::nullptr_t> = nullptr>
  Eigen::AlignedBox2i getBoundingBox() const
  {
    Eigen::Vector2i idxMin { std::numeric_limits<int>::max(),
      std::numeric_limits<int>::max() };
    Eigen::Vector2i idxMax { std::numeric_limits<int>::min(),
      std::numeric_limits<int>::min() };

    const int mapSizeX = this->getSizeX();
    const int mapSizeY = this->getSizeY();

    for (int y = 0; y < mapSizeY; ++y) {
      for (int x = 0; x < mapSizeX; ++x) {
        // Skip grid cells with no information (unknown occupancy probability)
        if (this->getCell(x, y).getValue() == 0.0f)
          continue;

        idxMin.x() = std::min(idxMin.x(), x);
        idxMin.y() = std::min(idxMin.y(), y);
        idxMax.x() = std::max(idxMax.x(), x);
        idxMax.y() = std::max(idxMax.y(), y);
      }
    }

    // If the grid map has no grid cells with known occupancy probabilities,
    // then the empty bounding box is returned
    return Eigen::AlignedBox2i { idxMin, idxMax };
  }

protected:
  ConcreteGridFunctions concreteGridFunctions;
  int currUpdateIndex;
  int currMarkOccIndex;
  int currMarkFreeIndex;
};

} // namespace hectorslam

#endif // HECTOR_SLAM_MAP_OCC_GRID_MAP_BASE_H
