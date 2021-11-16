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

#ifndef HECTOR_SLAM_SLAM_MAIN_MAP_REP_SINGLE_MAP_H
#define HECTOR_SLAM_SLAM_MAIN_MAP_REP_SINGLE_MAP_H

#include "map/GridMap.h"
#include "map/OccGridMapUtilConfig.h"
#include "matcher/ScanMatcher.h"
#include "slam_main/MapRepresentationInterface.h"
#include "util/DrawInterface.h"
#include "util/HectorDebugInfoInterface.h"

namespace hectorslam {

class MapRepSingleMap : public MapRepresentationInterface
{
public:
  MapRepSingleMap(float mapResolution,
                  DrawInterface* drawInterfaceIn,
                  HectorDebugInfoInterface* debugInterfaceIn) :
    gridMap(nullptr),
    gridMapUtil(nullptr),
    scanMatcher(nullptr)
  {
    this->gridMap = new GridMap(mapResolution, Eigen::Vector2i(1024, 1024),
      Eigen::Vector2f(20.0f, 20.0f));
    this->gridMapUtil = new OccGridMapUtilConfig<GridMap>(this->gridMap);
    this->scanMatcher = new ScanMatcher<OccGridMapUtilConfig<GridMap>>(
      drawInterfaceIn, debugInterfaceIn);
  }

  virtual ~MapRepSingleMap()
  {
    delete this->gridMap;
    delete this->gridMapUtil;
    delete this->scanMatcher;
  }

  virtual void reset()
  {
    this->gridMap->reset();
    this->gridMapUtil->resetCachedData();
  }

  virtual inline float getScaleToMap() const
  { return this->gridMap->getScaleToMap(); }

  virtual inline int getMapLevels() const { return 1; }
  virtual inline const GridMap& getGridMap(int mapLevel) const
  { return *this->gridMap; }

  virtual void onMapUpdated() { this->gridMapUtil->resetCachedData(); }

  virtual Eigen::Vector3f matchData(const Eigen::Vector3f& beginEstimateWorld,
                                    const DataContainer& dataContainer,
                                    Eigen::Matrix3f& covMatrix)
  {
    return this->scanMatcher->matchData(
      beginEstimateWorld, *this->gridMapUtil, dataContainer, covMatrix, 20);
  }

  virtual void updateByScan(const DataContainer& dataContainer,
                            const Eigen::Vector3f& robotPoseWorld)
  {
    this->gridMap->updateByScan(dataContainer, robotPoseWorld);
  }

protected:
  GridMap* gridMap;
  OccGridMapUtilConfig<GridMap>* gridMapUtil;
  ScanMatcher<OccGridMapUtilConfig<GridMap>>* scanMatcher;
};

} // namespace hectorslam

#endif // HECTOR_SLAM_SLAM_MAIN_MAP_REP_SINGLE_MAP_H
