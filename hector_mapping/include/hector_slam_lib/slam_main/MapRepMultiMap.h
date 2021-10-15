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

#ifndef _hectormaprepmultimap_h__
#define _hectormaprepmultimap_h__

#include <memory>

#include "MapRepresentationInterface.h"
#include "MapProcContainer.h"

#include "../map/GridMap.h"
#include "../map/OccGridMapUtilConfig.h"
#include "../matcher/ScanMatcher.h"

#include "../util/DrawInterface.h"
#include "../util/HectorDebugInfoInterface.h"

namespace hectorslam{

class MapRepMultiMap : public MapRepresentationInterface
{
public:
  // Type declaration for convenience
  using ConcreteScanMatcher = ScanMatcher<OccGridMapUtilConfig<GridMap>>;

  MapRepMultiMap(float mapResolution, int mapSizeX, int mapSizeY, unsigned int numDepth, const Eigen::Vector2f& startCoords, DrawInterface* drawInterfaceIn, HectorDebugInfoInterface* debugInterfaceIn)
  {
    //unsigned int numDepth = 3;
    Eigen::Vector2i resolution(mapSizeX, mapSizeY);

    float totalMapSizeX = mapResolution * static_cast<float>(mapSizeX);
    float mid_offset_x = totalMapSizeX * startCoords.x();

    float totalMapSizeY = mapResolution * static_cast<float>(mapSizeY);
    float mid_offset_y = totalMapSizeY * startCoords.y();

    this->scanMatcher = std::make_shared<ConcreteScanMatcher>(
      drawInterfaceIn, debugInterfaceIn);

    for (unsigned int i = 0; i < numDepth; ++i){
      std::cout << "HectorSM map lvl " << i << ": cellLength: " << mapResolution << " res x:" << resolution.x() << " res y: " << resolution.y() << "\n";
      auto gridMap = std::make_unique<GridMap>(
        mapResolution, resolution, Eigen::Vector2f(mid_offset_x, mid_offset_y));
      auto gridMapUtil = std::make_unique<OccGridMapUtilConfig<GridMap>>(
        gridMap.get());
      mapContainer.emplace_back(std::move(gridMap), std::move(gridMapUtil));

      resolution /= 2;
      mapResolution*=2.0f;
    }

    dataContainers.resize(numDepth-1);
  }

  virtual ~MapRepMultiMap()
  {
  }

  virtual void reset()
  {
    for (auto& map : this->mapContainer) {
      map.gridMap->reset();
      map.gridMapUtil->resetCachedData();
    }
  }

  virtual float getScaleToMap() const
  { return this->mapContainer[0].gridMap->getScaleToMap(); }

  virtual int getMapLevels() const { return mapContainer.size(); };
  virtual const GridMap& getGridMap(int mapLevel) const
  { return *this->mapContainer[mapLevel].gridMap; }

  virtual void addMapMutex(int i, MapLockerInterface* mutex)
  { this->mapContainer[i].mapMutex.reset(mutex); }

  inline MapLockerInterface* getMapMutex(int i)
  { return this->mapContainer[i].mapMutex.get(); }

  virtual void onMapUpdated()
  {
    for (auto& map : this->mapContainer)
      map.gridMapUtil->resetCachedData();
  }

  virtual Eigen::Vector3f matchData(const Eigen::Vector3f& beginEstimateWorld,
                                    const DataContainer& dataContainer,
                                    Eigen::Matrix3f& covMatrix)
  {
    const std::size_t size = this->mapContainer.size();

    Eigen::Vector3f poseEstimate = beginEstimateWorld;

    for (int index = size - 1; index >= 0; --index) {
      auto& map = this->mapContainer[index];
      if (index == 0) {
        poseEstimate = this->scanMatcher->matchData(
          poseEstimate, *map.gridMapUtil, dataContainer, covMatrix, 5);
      } else {
        const float scale = 1.0f / std::pow(2.0f, static_cast<float>(index));
        this->dataContainers[index - 1].setFrom(dataContainer, scale);
        poseEstimate = this->scanMatcher->matchData(
          poseEstimate, *map.gridMapUtil,
          this->dataContainers[index - 1], covMatrix, 3);
      }
    }
    return poseEstimate;
  }

  virtual void updateByScan(const DataContainer& dataContainer,
                            const Eigen::Vector3f& robotPoseWorld)
  {
    const std::size_t size = this->mapContainer.size();

    for (std::size_t i = 0; i < size; ++i) {
      auto& map = this->mapContainer[i];
      const auto& scan = (i == 0) ? dataContainer : this->dataContainers[i - 1];

      if (map.mapMutex != nullptr)
        map.mapMutex->lockMap();

      map.gridMap->updateByScan(scan, robotPoseWorld);

      if (map.mapMutex != nullptr)
        map.mapMutex->unlockMap();
    }
  }

  virtual void setUpdateFactorFree(float free_factor)
  {
    for (auto& map : this->mapContainer)
      map.gridMap->setUpdateFreeFactor(free_factor);
  }

  virtual void setUpdateFactorOccupied(float occupied_factor)
  {
    for (auto& map : this->mapContainer)
      map.gridMap->setUpdateOccupiedFactor(occupied_factor);
  }

protected:
  std::vector<MapProcContainer> mapContainer;
  std::vector<DataContainer> dataContainers;
  std::shared_ptr<ConcreteScanMatcher> scanMatcher;
};

}

#endif
