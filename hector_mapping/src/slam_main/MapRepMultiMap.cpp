
// MapRepMultiMap.cpp

#include "slam_main/MapRepMultiMap.h"

namespace hectorslam {

// Constructor
MapRepMultiMap::MapRepMultiMap(
  float mapResolution,
  const int mapSizeX, const int mapSizeY,
  const unsigned int numDepth, const Eigen::Vector2f& startCoords,
  ScanMatchCallback scanMatchCallback,
  DrawInterface* pDrawInterface,
  HectorDebugInfoInterface* pDebugInterface) :
  mScanMatchCallback(scanMatchCallback)
{
  // unsigned int numDepth = 3;
  Eigen::Vector2i resolution { mapSizeX, mapSizeY };

  const float totalMapSizeX = mapResolution * static_cast<float>(mapSizeX);
  const float midOffsetX = totalMapSizeX * startCoords.x();
  const float totalMapSizeY = mapResolution * static_cast<float>(mapSizeY);
  const float midOffsetY = totalMapSizeY * startCoords.y();
  const Eigen::Vector2f midOffset { midOffsetX, midOffsetY };

  for (unsigned int i = 0; i < numDepth; ++i) {
    std::cout << "HectorSM map lvl " << i << ": "
              << "cellLength: " << mapResolution << ' '
              << "res x: " << resolution.x() << ' '
              << "res y: " << resolution.y() << '\n';
    auto gridMap = std::make_unique<GridMap>(
      mapResolution, resolution, midOffset);
    auto gridMapUtil = std::make_unique<GridMapUtil>(gridMap.get());
    this->mMapContainer.emplace_back(
      std::move(gridMap), std::move(gridMapUtil));

    resolution /= 2;
    mapResolution *= 2.0f;
  }

  this->mDataContainers.resize(numDepth - 1);
}

void MapRepMultiMap::reset()
{
  for (auto& map : this->mMapContainer) {
    map.gridMap->reset();
    map.gridMapUtil->resetCachedData();
  }
}

void MapRepMultiMap::onMapUpdated()
{
  for (auto& map : this->mMapContainer)
    map.gridMapUtil->resetCachedData();
}

Eigen::Vector3f MapRepMultiMap::matchData(
  const Eigen::Vector3f& beginEstimateWorld,
  const DataContainer& dataContainer,
  Eigen::Matrix3f& covMatrix)
{
  const std::size_t size = this->mMapContainer.size();

  Eigen::Vector3f poseEstimate = beginEstimateWorld;

  for (int index = size - 1; index >= 0; --index) {
    auto& map = this->mMapContainer[index];

    // Scale the scan point coordinates for the coarser grid maps
    if (index != 0) {
      const float scale = 1.0f / std::pow(2.0f, static_cast<float>(index));
      this->mDataContainers[index - 1].setFrom(dataContainer, scale);
    }

    const auto& scan = (index == 0) ? dataContainer :
      this->mDataContainers[index - 1];
    poseEstimate = this->mScanMatchCallback(
        poseEstimate, *map.gridMapUtil, scan, covMatrix, index);
  }

  return poseEstimate;
}

void MapRepMultiMap::updateByScan(
  const DataContainer& dataContainer,
  const Eigen::Vector3f& robotPoseWorld)
{
  const std::size_t size = this->mMapContainer.size();

  for (std::size_t i = 0; i < size; ++i) {
    auto& map = this->mMapContainer[i];
    const auto& scan = (i == 0) ? dataContainer : this->mDataContainers[i - 1];

    if (map.mapMutex != nullptr)
      map.mapMutex->lockMap();

    map.gridMap->updateByScan(scan, robotPoseWorld);

    if (map.mapMutex != nullptr)
      map.mapMutex->unlockMap();
  }
}

void MapRepMultiMap::setUpdateFactorFree(float freeFactor)
{
  for (auto& map : this->mMapContainer)
    map.gridMap->setUpdateFreeFactor(freeFactor);
}

void MapRepMultiMap::setUpdateFactorOccupied(float occupiedFactor)
{
  for (auto& map : this->mMapContainer)
    map.gridMap->setUpdateOccupiedFactor(occupiedFactor);
}

} // namespace hectorslam
