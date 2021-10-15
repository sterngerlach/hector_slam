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

#ifndef HECTOR_SLAM_SLAM_MAIN_MAP_REP_MULTI_MAP_H
#define HECTOR_SLAM_SLAM_MAIN_MAP_REP_MULTI_MAP_H

#include <functional>
#include <memory>
#include <Eigen/Core>

#include "map/GridMap.h"
#include "map/OccGridMapUtilConfig.h"
#include "scan/DataPointContainer.h"
#include "slam_main/MapRepresentationInterface.h"
#include "slam_main/MapProcContainer.h"
#include "util/DrawInterface.h"
#include "util/HectorDebugInfoInterface.h"

namespace hectorslam {

class MapRepMultiMap : public MapRepresentationInterface
{
public:
  // Type definition for the occupancy grid map
  using GridMapUtil = OccGridMapUtilConfig<GridMap>;
  // Type definition for the scan matcher callback which takes the initial
  // pose estimate, occupancy grid map, scan, reference to the resulting
  // pose covariance, and index of the map, and returns the pose estimate
  using ScanMatchCallback = std::function<
    Eigen::Vector3f(const Eigen::Vector3f&, GridMapUtil&,
      const DataContainer&, Eigen::Matrix3f&, const int)>;

  MapRepMultiMap(float mapResolution, int mapSizeX, int mapSizeY,
                 unsigned int numDepth, const Eigen::Vector2f& startCoords,
                 ScanMatchCallback scanMatchCallback,
                 DrawInterface* drawInterfaceIn,
                 HectorDebugInfoInterface* debugInterfaceIn);

  virtual ~MapRepMultiMap() { }

  virtual void reset() override;

  virtual float getScaleToMap() const override
  { return this->mMapContainer[0].gridMap->getScaleToMap(); }
  virtual int getMapLevels() const override
  { return this->mMapContainer.size(); };
  virtual const GridMap& getGridMap(int mapLevel) const override
  { return *this->mMapContainer[mapLevel].gridMap; }

  virtual void addMapMutex(int i, MapLockerInterface* mutex) override
  { this->mMapContainer[i].mapMutex.reset(mutex); }
  inline MapLockerInterface* getMapMutex(int i) override
  { return this->mMapContainer[i].mapMutex.get(); }

  virtual void onMapUpdated() override;

  virtual Eigen::Vector3f matchData(const Eigen::Vector3f& beginEstimateWorld,
                                    const DataContainer& dataContainer,
                                    Eigen::Matrix3f& covMatrix) override;

  virtual void updateByScan(const DataContainer& dataContainer,
                            const Eigen::Vector3f& robotPoseWorld) override;

  virtual void setUpdateFactorFree(float freeFactor) override;
  virtual void setUpdateFactorOccupied(float occupiedFactor) override;

protected:
  std::vector<MapProcContainer> mMapContainer;
  std::vector<DataContainer> mDataContainers;
  ScanMatchCallback mScanMatchCallback;
};

} // namespace hectorslam

#endif // HECTOR_SLAM_SLAM_MAIN_MAP_REP_MULTI_MAP_H
