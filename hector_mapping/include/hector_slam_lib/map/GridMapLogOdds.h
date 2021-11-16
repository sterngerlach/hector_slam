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

#ifndef HECTOR_SLAM_MAP_GRID_MAP_LOG_ODDS_H
#define HECTOR_SLAM_MAP_GRID_MAP_LOG_ODDS_H

#include <cmath>

/**
 * Provides a log odds of occupancy probability representation
 * for cells in a occupancy grid map.
 */
class LogOddsCell
{
public:
  inline void set(float val) { this->logOddsVal = val; }
  inline float getValue() const { return this->logOddsVal; }

  inline bool isOccupied() const { return this->logOddsVal > 0.0f; }
  inline bool isFree() const { return this->logOddsVal < 0.0f; }

  /**
   * Reset Cell to prior probability.
   */
  void resetGridCell()
  {
    this->logOddsVal = 0.0f;
    this->updateIndex = -1;
  }

public:
  // The log odds representation of occupancy probability.
  float logOddsVal;
  int updateIndex;
};

/**
 * Provides functions related to a log odds of occupancy probability
 * respresentation for cells in a occupancy grid map.
 */
class GridMapLogOddsFunctions
{
public:
  GridMapLogOddsFunctions()
  {
    this->setUpdateFreeFactor(0.4f);
    this->setUpdateOccupiedFactor(0.6f);
  }

  void updateSetOccupied(LogOddsCell& cell) const
  {
    if (cell.logOddsVal < 50.0f)
      cell.logOddsVal += this->logOddsOccupied;
  }

  void updateSetFree(LogOddsCell& cell) const
  {
    cell.logOddsVal += this->logOddsFree;
  }

  void updateUnsetFree(LogOddsCell& cell) const
  {
    cell.logOddsVal -= this->logOddsFree;
  }

  float getGridProbability(const LogOddsCell& cell) const
  {
    const float odds = std::exp(cell.logOddsVal);
    return odds / (odds + 1.0f);
  }

  inline void setUpdateFreeFactor(float factor)
  { this->logOddsFree = this->probToLogOdds(factor); }
  inline void setUpdateOccupiedFactor(float factor)
  { this->logOddsOccupied = this->probToLogOdds(factor); }

protected:
  inline float probToLogOdds(float prob)
  {
    const float odds = prob / (1.0f - prob);
    return std::log(odds);
  }

  // The log odds representation of probability used for
  // updating cells as occupied
  float logOddsOccupied;
  // The log odds representation of probability used for
  // updating cells as free
  float logOddsFree;
};

#endif // HECTOR_SLAM_MAP_GRID_MAP_LOG_ODDS_H
