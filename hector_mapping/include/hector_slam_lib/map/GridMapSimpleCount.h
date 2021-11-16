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

#ifndef HECTOR_SLAM_MAP_GRID_MAP_SIMPLE_COUNT_H
#define HECTOR_SLAM_MAP_GRID_MAP_SIMPLE_COUNT_H

/**
 * Provides a (very) simple count based representation of occupancy
 */
class SimpleCountCell
{
public:
  inline void set(float val) { this->simpleOccVal = val; }
  inline float getValue() const { return this->simpleOccVal; }

  inline bool isOccupied() const { return this->simpleOccVal > 0.5f; }
  inline bool isFree() const { return this->simpleOccVal < 0.5f; }

  /**
   * Reset Cell to prior probability.
   */
  void resetGridCell()
  {
    this->simpleOccVal = 0.5f;
    this->updateIndex = -1;
  }

public:
  // The log odds representation of occupancy probability.
  float simpleOccVal;
  int updateIndex;
};

/**
 * Provides functions related to a log odds of occupancy probability
 * respresentation for cells in a occupancy grid map.
 */
class GridMapSimpleCountFunctions
{
public:
  GridMapSimpleCountFunctions()
  {
    this->updateFreeVal = -0.10f;
    this->updateOccVal = 0.15f;

    this->updateFreeLimit = -this->updateFreeVal
      + this->updateFreeVal / 100.0f;
    this->updateOccLimit = 1.0f - (this->updateOccVal
      + this->updateOccVal / 100.0f);
  }

  void updateSetOccupied(SimpleCountCell& cell) const
  {
    if (cell.simpleOccVal < this->updateOccLimit)
      cell.simpleOccVal += this->updateOccVal;
  }

  void updateSetFree(SimpleCountCell& cell) const
  {
    if (cell.simpleOccVal > this->updateFreeLimit)
      cell.simpleOccVal += this->updateFreeVal;
  }

  void updateUnsetFree(SimpleCountCell& cell) const
  {
    cell.simpleOccVal -= this->updateFreeVal;
  }

  inline float getGridProbability(const SimpleCountCell& cell) const
  { return cell.simpleOccVal; }

protected:
  float updateFreeVal;
  float updateOccVal;
  float updateFreeLimit;
  float updateOccLimit;
};

#endif // HECTOR_SLAM_MAP_GRID_MAP_SIMPLE_COUNT_H
