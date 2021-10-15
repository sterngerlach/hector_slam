
// ScanMatcherCorrelative.cpp

#include "matcher/ScanMatcherCorrelative.hpp"
#include "util/Assert.hpp"

namespace hectorslam {

// Constructor
ScanMatcherCorrelative::ScanMatcherCorrelative(
  const int coarseMapResolution,
  const Eigen::Vector3f& searchWindow,
  const float angleSearchStepMin,
  const float angleSearchStepMax,
  DrawInterface* pDrawInterface,
  HectorDebugInfoInterface* pDebugInterface) :
  mCoarseMapResolution(coarseMapResolution),
  mSearchWindow(searchWindow),
  mAngleSearchStepMin(angleSearchStepMin),
  mAngleSearchStepMax(angleSearchStepMax),
  mDrawInterface(pDrawInterface),
  mDebugInterface(pDebugInterface)
{
  Assert(coarseMapResolution > 1);
  Assert(searchWindow[0] >= 0.0f);
  Assert(searchWindow[1] >= 0.0f);
  Assert(searchWindow[2] >= 0.0f);
  Assert(angleSearchStepMin > 0.0f);
  Assert(angleSearchStepMax > 0.0f);
  Assert(angleSearchStepMax >= angleSearchStepMin);
}

// Match scan to grid map given an initial world pose estimate
Eigen::Vector3f ScanMatcherCorrelative::MatchScans(
  const Eigen::Vector3f& initialWorldPose,
  OccGridMapUtilConfig<GridMap>& gridMapUtil,
  const DataContainer& dataContainer,
  Eigen::Matrix3f& covMatrix,
  const float scoreMin, const float correspondenceRatioMin)
{
  // `scoreMin` and `correspondenceRatioMin` are the minimum score (averaged
  // occupancy probability) and the ratio of matching correspondences
  // to accept the estimated pose

  if (dataContainer.getSize() <= 0)
    return initialWorldPose;

  // Resize and clear the coarse grid map
  this->mCoarseMap.setMapSize(Eigen::Vector2i(
    gridMapUtil.getMapSizeX(), gridMapUtil.getMapSizeY()));
  this->mCoarseMap.resetCache();

  // Compute the maximum scan range (in the map coordinate frame)
  std::vector<float> scanRanges;
  scanRanges.reserve(dataContainer.getSize());
  std::transform(dataContainer.getVector().begin(),
    dataContainer.getVector().end(),
    std::back_inserter(scanRanges),
    [](const Eigen::Vector2f& point) { return point.squaredNorm(); });
  const float scanRangeMax = std::sqrt(*std::max_element(
    scanRanges.begin(), scanRanges.end()));
  // Determine the search step (in the map coordinate frame)
  const float stepTheta = std::min(this->mAngleSearchStepMax,
    std::max(this->mAngleSearchStepMin,
      std::acos(1.0f - 0.5f / scanRangeMax / scanRangeMax)));
  const Eigen::Vector3f step { 1.0f, 1.0f, stepTheta };
  // Scale the search window to the map coordinate frame, since `mSearchWindow`
  // is in the world coordinate frame
  const float scaleWorldToMap = gridMapUtil.getScaleToMap();
  const Eigen::Vector3f searchWindowMap =
    Eigen::AlignedScaling3f(scaleWorldToMap, scaleWorldToMap, 1.0f)
      * this->mSearchWindow;
  // Determine the search window size
  const Eigen::Vector3i windowSize =
    (0.5f * searchWindowMap.array() / step.array()).ceil().cast<int>();

  // Perform the correlative scan matching
  float scoreMax = scoreMin;
  Eigen::Vector3i bestEstimate = -windowSize;
  // Set the number of the skipped and processed solution candidates
  int numOfSkipped = 0;
  int numOfProcessed = 0;

  // Compute the discrete grid cell indices for scans
  std::vector<Eigen::Vector2i> indices;
  indices.reserve(dataContainer.getSize());

  // Compute the pose in the map coordinate frame
  const Eigen::Vector3f initialMapPose =
    gridMapUtil.getMapCoordsPose(initialWorldPose);

  // Find the best estimate from the discrete search window
  for (int t = -windowSize[2]; t <= windowSize[2]; ++t) {
    // Compute the discrete grid cell indices for the current rotation angle
    const Eigen::Vector3f mapPose {
      initialMapPose[0], initialMapPose[1], initialMapPose[2] + step[2] * t };
    this->ComputeMapIndices(mapPose, gridMapUtil, dataContainer, indices);

    // For a given rotation angle, projected scan points (discrete grid cell
    // indices) are related by pure translation along x or y search directions
    for (int y = -windowSize[1]; y <= windowSize[1];
         y += this->mCoarseMapResolution) {
      for (int x = -windowSize[0]; x <= windowSize[0];
           x += this->mCoarseMapResolution) {
        // Evaluate the score using the coarse grid map
        const Eigen::Vector2i offset { x, y };
        const float normalizedScore =
          this->ComputeScoreCoarseMap(gridMapUtil, indices, offset);

        // Skip the current estimate if the score is less than the threshold
        if (normalizedScore <= scoreMax) {
          ++numOfSkipped;
          continue;
        }

        // Evaluate the score using the original grid map
        const Eigen::Vector3i startEstimate { x, y, t };
        this->ComputeScoreFineMap(
          gridMapUtil, indices, startEstimate, bestEstimate, scoreMax);
        ++numOfProcessed;
      }
    }
  }

  // Compute the best pose (in the world coordinate frame)
  const bool foundEstimate = scoreMax > scoreMin;
  const Eigen::Vector3f bestMapPose =
    initialMapPose.array() + step.array() * bestEstimate.cast<float>().array();
  const Eigen::Vector3f bestWorldPose =
    gridMapUtil.getWorldCoordsPose(bestMapPose);
  // Compute the covariance matrix (in the world coordinate frame)
  const Eigen::Matrix3f covMatrixMap =
    gridMapUtil.getCovarianceForPose(bestMapPose, dataContainer);
  const Eigen::Matrix3f covMatrixWorld =
    gridMapUtil.getCovMatrixWorldCoords(covMatrixMap);
  // Set the covariance matrix (in the map coordinate frame)
  covMatrix = covMatrixMap;

  return bestWorldPose;
}

// Compute the grid cell indices for scan points
void ScanMatcherCorrelative::ComputeMapIndices(
  const Eigen::Vector3f& mapPose,
  OccGridMapUtilConfig<GridMap>& gridMapUtil,
  const DataContainer& dataContainer,
  std::vector<Eigen::Vector2i>& indices) const
{
  // Compute the discrete grid cell indices for the given pose `worldPose`
  indices.clear();

  const Eigen::Affine2f mapPoseTransform =
    gridMapUtil.getTransformForState(mapPose);
  const int numOfPoints = dataContainer.getSize();

  for (int i = 0; i < numOfPoints; ++i) {
    const Eigen::Vector2f& point = dataContainer.getVecEntry(i);
    const Eigen::Vector2f mapPoint = mapPoseTransform * point;
    indices.push_back(mapPoint.cast<int>());
  }
}

// Compute the score using the coarse grid map
float ScanMatcherCorrelative::ComputeScoreCoarseMap(
  OccGridMapUtilConfig<GridMap>& gridMapUtil,
  const std::vector<Eigen::Vector2i>& indices,
  const Eigen::Vector2i& offset)
{
  float score = 0.0f;
  const int numOfPoints = static_cast<int>(indices.size());
  const int mapSizeX = gridMapUtil.getMapSizeX();
  const int mapSizeY = gridMapUtil.getMapSizeY();

  for (int i = 0; i < numOfPoints; ++i) {
    const Eigen::Vector2i mapIdx = indices[i] + offset;

    if (mapIdx.x() < 0 || mapIdx.x() >= mapSizeX ||
        mapIdx.y() < 0 || mapIdx.y() >= mapSizeY)
      continue;

    const int flatIdx = mapIdx.y() * mapSizeX + mapIdx.x();
    float probability = 0.0f;

    // If the grid cell at the coarse map is not available, then compute its
    // occupancy probability using its neighboring grid cells
    if (!this->mCoarseMap.containsCachedData(flatIdx, probability)) {
      const int rows = std::min(this->mCoarseMapResolution,
        std::max(0, mapSizeY - mapIdx.y()));
      const int cols = std::min(this->mCoarseMapResolution,
        std::max(0, mapSizeX - mapIdx.x()));
      Eigen::Vector2i idx = mapIdx;

      for (int y = 0; y < rows; ++y, ++idx.y())
        for (int x = 0; x < cols; ++x, ++idx.x())
          probability = std::max(probability,
            gridMapUtil.getUnfilteredGridPoint(idx.y() * mapSizeX + idx.x()));

      this->mCoarseMap.cacheData(flatIdx, probability);
    }

    score += probability;
  }

  const float normalizedScore = score / numOfPoints;
  return normalizedScore;
}

// Compute the score using the original grid map
void ScanMatcherCorrelative::ComputeScoreFineMap(
  OccGridMapUtilConfig<GridMap>& gridMapUtil,
  const std::vector<Eigen::Vector2i>& indices,
  const Eigen::Vector3i& startEstimate,
  Eigen::Vector3i& bestEstimate, float& scoreMax)
{
  const int numOfPoints = static_cast<int>(indices.size());
  const int mapSizeX = gridMapUtil.getMapSizeX();
  const int mapSizeY = gridMapUtil.getMapSizeY();

  for (int y = 0; y < this->mCoarseMapResolution; ++y) {
    for (int x = 0; x < this->mCoarseMapResolution; ++x) {
      const Eigen::Vector2i offset {
        startEstimate[0] + x, startEstimate[1] + y };
      float score = 0.0f;

      for (int i = 0; i < numOfPoints; ++i) {
        const Eigen::Vector2i mapIdx = indices[i] + offset;

        if (mapIdx.x() < 0 || mapIdx.x() >= mapSizeX ||
            mapIdx.y() < 0 || mapIdx.y() >= mapSizeY)
          continue;

        const int flatIdx = mapIdx.y() * mapSizeX + mapIdx.x();
        const float probability = gridMapUtil.getUnfilteredGridPoint(flatIdx);
        score += probability;
      }

      // Normalize the score
      score /= numOfPoints;

      // Update the best score and estimate
      if (score > scoreMax) {
        scoreMax = score;
        bestEstimate.head<2>() = offset;
        bestEstimate[2] = startEstimate[2];
      }
    }
  }
}

} // namespace hectorslam