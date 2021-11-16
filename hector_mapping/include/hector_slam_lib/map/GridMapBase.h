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

#ifndef HECTOR_SLAM_MAP_GRID_MAP_BASE_H
#define HECTOR_SLAM_MAP_GRID_MAP_BASE_H

#include <Eigen/Geometry>
#include <Eigen/LU>

#include "map/MapDimensionProperties.h"

namespace hectorslam {

/**
 * GridMapBase provides basic grid map functionality
 * (creates grid, provides transformation from/to world coordinates).
 * It serves as the base class for different map representations
 * that may extend it's functionality.
 */
template<typename ConcreteCellType>
class GridMapBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * Indicates if given x and y are within map bounds
   * @return True if coordinates are within map bounds
   */
  inline bool hasGridValue(int x, int y) const
  { return (x >= 0) && (y >= 0) &&
           (x < this->getSizeX()) && (y < this->getSizeY()); }

  inline const Eigen::Vector2i& getMapDimensions() const
  { return this->mapDimensionProperties.getMapDimensions(); }
  inline int getSizeX() const { return mapDimensionProperties.getSizeX(); }
  inline int getSizeY() const { return mapDimensionProperties.getSizeY(); }

  inline bool pointOutOfMapBounds(const Eigen::Vector2f& pointMapCoords) const
  { return this->mapDimensionProperties.pointOutOfMapBounds(pointMapCoords); }

  virtual void reset() { this->clear(); }

  /**
   * Resets the grid cell values by using the resetGridCell() function.
   */
  void clear()
  {
    const int size = this->getSizeX() * this->getSizeY();
    for (int i = 0; i < size; ++i)
      this->mapArray[i].resetGridCell();
  }

  inline const MapDimensionProperties& getMapDimProperties() const
  { return this->mapDimensionProperties; }

  /**
   * Constructor, creates grid representation and transformations.
   */
  GridMapBase(const float mapResolution,
              const Eigen::Vector2i& size,
              const Eigen::Vector2f& offset) :
    mapArray(nullptr),
    lastUpdateIndex(-1)
  {
    this->setMapGridSize(size);
    this->sizeX = size[0];
    this->setMapTransformation(offset, mapResolution);
    this->clear();
  }

  /**
   * Destructor
   */
  virtual ~GridMapBase() { this->deleteArray(); }

  /**
   * Allocates memory for the two dimensional pointer array for
   * map representation.
   */
  void allocateArray(const Eigen::Vector2i& newMapDims)
  {
    const int size = newMapDims.x() * newMapDims.y();
    this->mapArray = new ConcreteCellType[size];
    this->mapDimensionProperties.setMapCellDims(newMapDims);
  }

  void deleteArray()
  {
    if (this->mapArray != nullptr) {
      delete[] this->mapArray;
      this->mapArray = nullptr;
      this->mapDimensionProperties.setMapCellDims(Eigen::Vector2i(-1, -1));
    }
  }

  inline ConcreteCellType& getCell(int x, int y)
  { return this->mapArray[y * this->sizeX + x]; }
  inline const ConcreteCellType& getCell(int x, int y) const
  { return this->mapArray[y * this->sizeX + x]; }

  inline ConcreteCellType& getCell(int index)
  { return this->mapArray[index]; }
  inline const ConcreteCellType& getCell(int index) const
  { return this->mapArray[index]; }

  void setMapGridSize(const Eigen::Vector2i& newMapDims)
  {
    if (newMapDims != this->mapDimensionProperties.getMapDimensions()) {
      this->deleteArray();
      this->allocateArray(newMapDims);
      this->reset();
    }
  }

  /**
   * Copy Constructor, only needed if pointer members are present.
   */
  GridMapBase(const GridMapBase& other)
  {
    this->allocateArray(other.getMapDimensions());
    *this = other;
  }

  /**
   * Assignment operator, only needed if pointer members are present.
   */
  GridMapBase& operator=(const GridMapBase& other)
  {
    if (this->mapDimensionProperties != other.mapDimensionProperties)
      this->setMapGridSize(other.mapDimensionProperties.getMapDimensions());

    this->mapDimensionProperties = other.mapDimensionProperties;

    this->worldTmap = other.worldTmap;
    this->mapTworld = other.mapTworld;
    this->worldTmap3D = other.worldTmap3D;

    this->scaleToMap = other.scaleToMap;

    // @todo potential resize
    const int size = this->getSizeX() * this->getSizeY();
    const std::size_t concreteCellSize = sizeof(ConcreteCellType);
    std::memcpy(this->mapArray, other.mapArray, size * concreteCellSize);

    return *this;
  }

  /**
   * Returns the world coordinates for the given map coords.
   */
  inline Eigen::Vector2f getWorldCoords(const Eigen::Vector2f& mapCoords) const
  { return this->worldTmap * mapCoords; }

  /**
   * Returns the map coordinates for the given world coords.
   */
  inline Eigen::Vector2f getMapCoords(const Eigen::Vector2f& worldCoords) const
  { return this->mapTworld * worldCoords; }

  /**
   * Returns the world pose for the given map pose.
   */
  inline Eigen::Vector3f getWorldCoordsPose(
    const Eigen::Vector3f& mapPose) const
  {
    const Eigen::Vector2f worldCoords { this->worldTmap * mapPose.head<2>() };
    return Eigen::Vector3f { worldCoords[0], worldCoords[1], mapPose[2] };
  }

  /**
   * Returns the map pose for the given world pose.
   */
  inline Eigen::Vector3f getMapCoordsPose(
    const Eigen::Vector3f& worldPose) const
  {
    const Eigen::Vector2f mapCoords { this->mapTworld * worldPose.head<2>() };
    return Eigen::Vector3f { mapCoords[0], mapCoords[1], worldPose[2] };
  }

  void setDimensionProperties(const Eigen::Vector2f& topLeftOffsetIn,
                              const Eigen::Vector2i& mapDimensionsIn,
                              const float cellLengthIn)
  { this->setDimensionProperties(MapDimensionProperties(
      topLeftOffsetIn, mapDimensionsIn, cellLengthIn)); }

  void setDimensionProperties(const MapDimensionProperties& newMapDimProps)
  {
    // Grid map cell number has changed
    if (!newMapDimProps.hasEqualDimensionProperties(
        this->mapDimensionProperties))
      this->setMapGridSize(newMapDimProps.getMapDimensions());

    // Grid map transformation/cell size has changed
    if (!newMapDimProps.hasEqualTransformationProperties(
        this->mapDimensionProperties))
      this->setMapTransformation(newMapDimProps.getTopLeftOffset(),
                                 newMapDimProps.getCellLength());
  }

  /**
   * Set the map transformations
   * @param xWorld The origin of the map coordinate system on the x axis
   * in world coordinates
   * @param yWorld The origin of the map coordinate system on the y axis
   * in world coordinates
   * @param The cell length of the grid map
   */
  void setMapTransformation(const Eigen::Vector2f& topLeftOffset,
                            const float cellLength)
  {
    this->mapDimensionProperties.setCellLength(cellLength);
    this->mapDimensionProperties.setTopLeftOffset(topLeftOffset);

    this->scaleToMap = 1.0f / cellLength;

    this->mapTworld = Eigen::AlignedScaling2f(
      this->scaleToMap, this->scaleToMap)
      * Eigen::Translation2f(topLeftOffset[0], topLeftOffset[1]);
    this->worldTmap3D = Eigen::AlignedScaling3f(
      this->scaleToMap, this->scaleToMap, 1.0f)
      * Eigen::Translation3f(topLeftOffset[0], topLeftOffset[1], 0);
    this->worldTmap3D = this->worldTmap3D.inverse();
    this->worldTmap = this->mapTworld.inverse();
  }

  /**
   * Returns the scale factor for one unit in world coords to one unit
   * in map coords.
   * @return The scale factor
   */
  inline float getScaleToMap() const { return this->scaleToMap; }

  /**
   * Returns the cell edge length of grid cells in millimeters.
   * @return the cell edge length in millimeters.
   */
  inline float getCellLength() const
  { return this->mapDimensionProperties.getCellLength(); }

  /**
   * Returns a reference to the homogenous 2D transform from map
   * to world coordinates.
   * @return The homogenous 2D transform.
   */
  inline const Eigen::Affine2f& getWorldTmap() const
  { return this->worldTmap; }

  /**
   * Returns a reference to the homogenous 3D transform from map
   * to world coordinates.
   * @return The homogenous 3D transform.
   */
  inline const Eigen::Affine3f& getWorldTmap3D() const
  { return this->worldTmap3D; }

  /**
   * Returns a reference to the homogenous 2D transform from world
   * to map coordinates.
   * @return The homogenous 2D transform.
   */
  inline const Eigen::Affine2f& getMapTworld() const
  { return this->mapTworld; }

  inline void setUpdated() { this->lastUpdateIndex++; }
  inline int getUpdateIndex() const { return this->lastUpdateIndex; }

protected:
  // Map representation used with plain pointer array.
  ConcreteCellType *mapArray;

  // Scaling factor from world to map.
  float scaleToMap;

  // Homogenous 2D transform from map to world coordinates.
  Eigen::Affine2f worldTmap;
  // Homogenous 3D transform from map to world coordinates.
  Eigen::Affine3f worldTmap3D;
  // Homogenous 2D transform from world to map coordinates.
  Eigen::Affine2f mapTworld;

  MapDimensionProperties mapDimensionProperties;
  int sizeX;

private:
  int lastUpdateIndex;
};

} // namespace hectorslam

#endif // HECTOR_SLAM_MAP_GRID_MAP_BASE_H
