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

#ifndef HECTOR_SLAM_SCAN_DATA_POINT_CONTAINER_H
#define HECTOR_SLAM_SCAN_DATA_POINT_CONTAINER_H

#include <algorithm>
#include <vector>
#include <Eigen/Core>

namespace hectorslam {

template<typename DataPointType>
class DataPointContainer
{
public:
  DataPointContainer(int size = 1000)
  {
    this->dataPoints.reserve(size);
  }

  void setFrom(const DataPointContainer& other, float factor)
  {
    this->origo = other.getOrigo() * factor;
    this->dataPoints = other.dataPoints;
    std::for_each(this->dataPoints.begin(), this->dataPoints.end(),
                  [factor](DataPointType& point) { point *= factor; });
  }

  inline void add(const DataPointType& dataPoint)
  { this->dataPoints.push_back(dataPoint); }

  inline void clear() { this->dataPoints.clear(); }

  inline int getSize() const
  { return static_cast<int>(this->dataPoints.size()); }

  inline const DataPointType& getVecEntry(int index) const
  { return this->dataPoints[index]; }

  inline DataPointType getOrigo() const { return this->origo; }
  inline void setOrigo(const DataPointType& origoIn) { this->origo = origoIn; }

  // Get the reference to the underlying vector
  inline const std::vector<DataPointType>& getVector() const
  { return this->dataPoints; }

protected:
  std::vector<DataPointType> dataPoints;
  DataPointType origo;
};

typedef DataPointContainer<Eigen::Vector2f> DataContainer;

} // namespace hectorslam

#endif // HECTOR_SLAM_SCAN_DATA_POINT_CONTAINER_H
