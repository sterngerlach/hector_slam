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

#ifndef HECTOR_SLAM_MAP_DIMENSION_PROPERTIES_H
#define HECTOR_SLAM_MAP_DIMENSION_PROPERTIES_H

#include <Eigen/Core>

class MapDimensionProperties
{
public:
  MapDimensionProperties() :
    topLeftOffset(-1.0f, -1.0f),
    mapDimensions(-1, -1),
    cellLength(-1.0f) { }

  MapDimensionProperties(const Eigen::Vector2f& topLeftOffsetIn,
                         const Eigen::Vector2i& mapDimensionsIn,
                         const float cellLengthIn) :
    topLeftOffset(topLeftOffsetIn),
    mapDimensions(mapDimensionsIn),
    cellLength(cellLengthIn)
  {
    this->mapLimitsf = mapDimensionsIn.cast<float>().array() - 1.0f;
  }

  inline bool operator==(const MapDimensionProperties& other) const
  {
    return this->topLeftOffset == other.topLeftOffset &&
           this->mapDimensions == other.mapDimensions &&
           this->cellLength == other.cellLength;
  }

  inline bool operator!=(const MapDimensionProperties& other) const
  { return !this->operator==(other); }

  inline bool hasEqualDimensionProperties(
    const MapDimensionProperties& other) const
  {
    return this->mapDimensions == other.mapDimensions;
  }

  inline bool hasEqualTransformationProperties(
    const MapDimensionProperties& other) const
  {
     return this->topLeftOffset == other.topLeftOffset &&
            this->cellLength == other.cellLength;
  }

  inline bool pointOutOfMapBounds(const Eigen::Vector2f& coords) const
  {
    return coords[0] < 0.0f ||
           coords[1] < 0.0f ||
           coords[0] > this->mapLimitsf[0] ||
           coords[1] > this->mapLimitsf[1];
  }

  inline void setMapCellDims(const Eigen::Vector2i& newDims)
  {
    this->mapDimensions = newDims;
    this->mapLimitsf = newDims.cast<float>().array() - 2.0f;
  }

  inline void setTopLeftOffset(const Eigen::Vector2f& topLeftOffsetIn)
  {
    this->topLeftOffset = topLeftOffsetIn;
  }

  inline void setSizeX(int sX) { this->mapDimensions[0] = sX; }
  inline void setSizeY(int sY) { this->mapDimensions[1] = sY; }
  inline void setCellLength(float cl) { this->cellLength = cl; }

  inline const Eigen::Vector2f& getTopLeftOffset() const
  { return this->topLeftOffset; }
  inline const Eigen::Vector2i& getMapDimensions() const
  { return this->mapDimensions; }
  inline int getSizeX() const { return this->mapDimensions[0]; }
  inline int getSizeY() const { return this->mapDimensions[1]; }
  inline float getCellLength() const { return this->cellLength; }

protected:
  Eigen::Vector2f topLeftOffset;
  Eigen::Vector2i mapDimensions;
  Eigen::Vector2f mapLimitsf;
  float cellLength;
};

#endif // HECTOR_SLAM_MAP_DIMENSION_PROPERTIES_H
