
// ScanMatcherFPGA.cpp

#include "matcher/ScanMatcherFPGA.hpp"
#include "util/Assert.hpp"
#include "util/DataConversion.hpp"
#include "util/Parameter.hpp"
#include "util/Timer.hpp"
#include "util/UtilFunctions.h"

#define RETURN_FALSE_IF_FAILED(call) if (!(call)) return false;
#define RETURN_NULLPTR_IF_FAILED(call) if (!(call)) return nullptr;

namespace hectorslam {

// Load the members from the given ros::NodeHandle
bool ScanMatcherIPConfig::FromNodeHandle(
  ros::NodeHandle& nh, ScanMatcherIPConfig& ipConfig)
{
  RETURN_FALSE_IF_FAILED(GetParamFromNodeHandle(
    nh, "num_of_scans_max", ipConfig.mNumOfScansMax))
  RETURN_FALSE_IF_FAILED(GetParamFromNodeHandle(
    nh, "map_size_x_max", ipConfig.mMapSizeXMax))
  RETURN_FALSE_IF_FAILED(GetParamFromNodeHandle(
    nh, "map_size_y_max", ipConfig.mMapSizeYMax))
  RETURN_FALSE_IF_FAILED(GetParamFromNodeHandle(
    nh, "coarse_map_resolution", ipConfig.mCoarseMapResolution))
  RETURN_FALSE_IF_FAILED(GetParamFromNodeHandle(
    nh, "bits_per_value", ipConfig.mBitsPerValue))
  RETURN_FALSE_IF_FAILED(GetParamFromNodeHandle(
    nh, "grid_cells_per_chunk", ipConfig.mGridCellsPerChunk))

  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_base_address", ipConfig.mAxiLiteSBaseAddress))
  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_address_range", ipConfig.mAxiLiteSAddressRange))
  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_ap_ctrl", ipConfig.mAxiLiteSApCtrl))
  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_gie", ipConfig.mAxiLiteSGIE))
  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_ier", ipConfig.mAxiLiteSIER))
  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_isr", ipConfig.mAxiLiteSISR))

  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_num_of_scans", ipConfig.mAxiLiteSNumOfScans))
  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_scan_range_max", ipConfig.mAxiLiteSScanRangeMax))
  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_score_threshold", ipConfig.mAxiLiteSScoreThreshold))

  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_pose_x", ipConfig.mAxiLiteSPoseX))
  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_pose_y", ipConfig.mAxiLiteSPoseY))
  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_pose_theta", ipConfig.mAxiLiteSPoseTheta))
  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_map_size_x", ipConfig.mAxiLiteSMapSizeX))
  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_map_size_y", ipConfig.mAxiLiteSMapSizeY))
  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_map_min_x", ipConfig.mAxiLiteSMapMinX))
  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_map_min_y", ipConfig.mAxiLiteSMapMinY))

  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_win_x", ipConfig.mAxiLiteSWinX))
  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_win_y", ipConfig.mAxiLiteSWinY))
  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_win_theta", ipConfig.mAxiLiteSWinTheta))
  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_step_x", ipConfig.mAxiLiteSStepX))
  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_step_y", ipConfig.mAxiLiteSStepY))
  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "axi_lite_s_step_theta", ipConfig.mAxiLiteSStepTheta))

  return true;
}

// Load the members from the given ros::NodeHandle
bool AxiDmaConfig::FromNodeHandle(
  ros::NodeHandle& nh, AxiDmaConfig& axiDmaConfig)
{
  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "base_address", axiDmaConfig.mBaseAddress))
  RETURN_FALSE_IF_FAILED(GetAddressFromNodeHandle(
    nh, "address_range", axiDmaConfig.mAddressRange))

  return true;
}

// Load the configuration settings and create a new instance
std::unique_ptr<ScanMatcherFPGA> ScanMatcherFPGA::Create(
  ros::NodeHandle& nh,
  DrawInterface* pDrawInterface,
  HectorDebugInfoInterface* pDebugInterface)
{
  ros::NodeHandle nhScanMatcher { nh, "scan_matcher_fpga" };
  ros::NodeHandle nhIPConfig { nhScanMatcher, "ip_config" };
  ros::NodeHandle nhAxiDmaConfig { nhScanMatcher, "axi_dma_config" };

  int coarseMapResolution;
  Eigen::Vector3f searchWindow;
  float angleSearchStepMin;
  float angleSearchStepMax;
  ScanMatcherIPConfig ipConfig;
  AxiDmaConfig axiDmaConfig;

  RETURN_NULLPTR_IF_FAILED(GetParamFromNodeHandle(
    nhScanMatcher, "coarse_map_resolution", coarseMapResolution))
  RETURN_NULLPTR_IF_FAILED(GetParamFromNodeHandle(
    nhScanMatcher, "search_window_x", searchWindow[0]))
  RETURN_NULLPTR_IF_FAILED(GetParamFromNodeHandle(
    nhScanMatcher, "search_window_y", searchWindow[1]))
  RETURN_NULLPTR_IF_FAILED(GetParamFromNodeHandle(
    nhScanMatcher, "search_window_theta", searchWindow[2]))
  RETURN_NULLPTR_IF_FAILED(GetParamFromNodeHandle(
    nhScanMatcher, "angle_search_step_min", angleSearchStepMin))
  RETURN_NULLPTR_IF_FAILED(GetParamFromNodeHandle(
    nhScanMatcher, "angle_search_step_max", angleSearchStepMax))

  RETURN_NULLPTR_IF_FAILED(
    ScanMatcherIPConfig::FromNodeHandle(nhIPConfig, ipConfig))
  RETURN_NULLPTR_IF_FAILED(
    AxiDmaConfig::FromNodeHandle(nhAxiDmaConfig, axiDmaConfig));

  return std::make_unique<ScanMatcherFPGA>(
    coarseMapResolution, searchWindow,
    angleSearchStepMin, angleSearchStepMax,
    ipConfig, axiDmaConfig, pDrawInterface, pDebugInterface);
}

// Constructor
ScanMatcherFPGA::ScanMatcherFPGA(
  const int coarseMapResolution,
  const Eigen::Vector3f& searchWindow,
  const float angleSearchStepMin,
  const float angleSearchStepMax,
  const ScanMatcherIPConfig& ipConfig,
  const AxiDmaConfig& axiDmaConfig,
  DrawInterface* pDrawInterface,
  HectorDebugInfoInterface* pDebugInterface) :
  mCoarseMapResolution(coarseMapResolution),
  mSearchWindow(searchWindow),
  mAngleSearchStepMin(angleSearchStepMin),
  mAngleSearchStepMax(angleSearchStepMax),
  mIPConfig(ipConfig),
  mAxiDmaConfig(axiDmaConfig),
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

  // Check the configurations for the scan matcher IP core
  Assert(this->mIPConfig.mNumOfScansMax > 0);
  Assert(this->mIPConfig.mMapSizeXMax > 0);
  Assert(this->mIPConfig.mMapSizeYMax > 0);
  Assert(this->mIPConfig.mCoarseMapResolution > 0);
  Assert(this->mIPConfig.mBitsPerValue > 0 &&
         this->mIPConfig.mBitsPerValue <= 8);
  Assert(this->mIPConfig.mGridCellsPerChunk == 8);

  // Initialize the scan matcher IP core
  this->mIPRegisters = std::make_unique<hw::MemoryMappedIO>(
    this->mIPConfig.mAxiLiteSBaseAddress,
    this->mIPConfig.mAxiLiteSAddressRange);
  // Initialize the AXI DMA IP core
  hw::SharedMemoryMappedIO axiDmaRegisters {
    this->mAxiDmaConfig.mBaseAddress,
    this->mAxiDmaConfig.mAddressRange };
  this->mAxiDma = std::make_unique<hw::AxiSimpleDMA>(axiDmaRegisters);

  // Initialize the contiguous memory to store the inputs
  this->InitializeCMAForInput();
  // Initialize the contiguous memory to store the outputs
  this->InitializeCMAForOutput();

  // Reset and halt the AXI DMA IP core
  this->mAxiDma->SendChannel().Reset();
  this->mAxiDma->RecvChannel().Reset();
  this->mAxiDma->SendChannel().Stop();
  this->mAxiDma->RecvChannel().Stop();
  // Start the AXI DMA transfer
  this->mAxiDma->SendChannel().Start();
  this->mAxiDma->RecvChannel().Start();
}

// Match scan to grid map given an initial world pose estimate
Eigen::Vector3f ScanMatcherFPGA::MatchScans(
  const Eigen::Vector3f& initialWorldPose,
  const OccGridMapUtilConfig<GridMap>& gridMapUtil,
  const DataContainer& dataContainer,
  const float scoreMin, const float correspondenceRatioMin,
  Eigen::Matrix3f* pCovMatrix,
  hector_mapping::ScanMatcherFPGAMetrics* pMetrics)
{
  if (dataContainer.getSize() <= 0)
    return initialWorldPose;

  Timer outerTimer;
  Timer timer;

  // `scaleMapToWorld` is same as the grid map resolution (e.g., 0.05) and
  // `scaleWorldToMap` is its reciprocal (e.g., 20.0)
  const float scaleMapToWorld = gridMapUtil.getCellLength();
  const float scaleWorldToMap = gridMapUtil.getScaleToMap();

  // Setup the scan matcher IP control registers
  ScanMatcherIPRegisters ipRegisters;

  // Compute the number of the scan points to transfer
  ipRegisters.mNumOfScans = std::min(this->mIPConfig.mNumOfScansMax,
    dataContainer.getSize());
  // Compute the quantized score threshold
  ipRegisters.mScoreMin = static_cast<int>(
    ((1 << this->mIPConfig.mBitsPerValue) - 1)
      * (scoreMin * ipRegisters.mNumOfScans));

  // Compute the maximum scan range (in the map coordinate frame)
  std::vector<float> scanRanges;
  scanRanges.reserve(dataContainer.getSize());
  std::transform(dataContainer.getVector().begin(),
    dataContainer.getVector().end(),
    std::back_inserter(scanRanges),
    [](const Eigen::Vector2f& point) { return point.squaredNorm(); });
  const float scanRangeMax = std::sqrt(*std::max_element(
    scanRanges.begin(), scanRanges.end()));
  // Compute the maximum valid scan range
  ipRegisters.mValidScanRangeMax = scanRangeMax + 0.1f;

  // Compute the search step (in the map coordinate frame)
  const float stepTheta = std::min(this->mAngleSearchStepMax,
    std::max(this->mAngleSearchStepMin,
      std::acos(1.0f - 0.5f / scanRangeMax / scanRangeMax)));
  const Eigen::Vector3f step { 1.0f, 1.0f, stepTheta };
  // Convert the scaled map coordinate frame
  ipRegisters.mSearchStep = Eigen::AlignedScaling3f(
    scaleMapToWorld, scaleMapToWorld, 1.0f) * step;
  // Scale the search window to the map coordinate frame,
  // since `mSearchWindow` is in the world coordinate frame
  const Eigen::Vector3f searchWindowMap =
    Eigen::AlignedScaling3f(scaleWorldToMap, scaleWorldToMap, 1.0f)
      * this->mSearchWindow;
  // Compute the search window size
  const Eigen::Vector3i desiredWindowSize =
    (0.5f * searchWindowMap.array() / step.array()).ceil().cast<int>();
  const Eigen::Vector3i windowSizeMin { 1, 1, 1 };
  const Eigen::Vector3i windowSizeMax { this->mIPConfig.mMapSizeXMax / 2,
    this->mIPConfig.mMapSizeYMax / 2, desiredWindowSize[2] };
  const Eigen::Vector3i halfWindowSize = windowSizeMax.array().min(
    windowSizeMin.array().max(desiredWindowSize.array()));
  ipRegisters.mWindowSize = halfWindowSize * 2;

  // Compute the initial pose (in the map coordinate frame)
  const Eigen::Vector3f initialMapPose =
    gridMapUtil.getMapCoordsPose(initialWorldPose);
  // Compute the minimum possible pose (in the map coordinate frame)
  const Eigen::Vector3f mapPoseMin = initialMapPose.array()
    - step.array() * halfWindowSize.cast<float>().array();
  // Convert to the scaled map coordinate frame
  ipRegisters.mPoseMin = Eigen::AlignedScaling3f(
    scaleMapToWorld, scaleMapToWorld, 1.0f) * mapPoseMin;

  // Compute the size and starting position of the grid map to transfer
  const Eigen::AlignedBox2i boundingBox = this->ComputeBoundingBox(
    gridMapUtil, initialMapPose);
  ipRegisters.mMapSize = boundingBox.sizes();
  // Convert to the scaled map coordinate frame
  ipRegisters.mMapPosMin = Eigen::AlignedScaling2f(
    scaleMapToWorld, scaleMapToWorld) * boundingBox.min().cast<float>();

  // Write the scan matcher IP control registers
  this->SetIPRegisters(ipRegisters);
  const int inputSetupTime = timer.ElapsedNanoseconds();
  timer.Start();

  // Start the scan matcher IP core
  this->StartIP();
  const int ipSetupTime = timer.ElapsedNanoseconds();
  timer.Start();

  // Transfer the scan through AXI4-Stream interface
  this->TransferScan(dataContainer, ipRegisters.mNumOfScans, scaleMapToWorld);
  const int scanTransferTime = timer.ElapsedNanoseconds();
  timer.Start();

  // Transfer the grid map through AXI4-Stream interface
  Eigen::Vector2i transferredMapSize;
  int numOfTransferredNumBlocks;
  this->TransferMap(gridMapUtil, boundingBox,
                    transferredMapSize, numOfTransferredNumBlocks);
  const int mapTransferTime = timer.ElapsedNanoseconds();
  timer.Start();

  // Receive the result from AXI4-Stream interface
  Eigen::Vector3i bestEstimate;
  int scoreMax;
  this->ReceiveResult(bestEstimate, scoreMax);
  const int ipOptimizationTime = timer.ElapsedNanoseconds();
  timer.Start();

  // Wait until the scan matcher IP core is in idle state
  this->WaitIP();
  const int ipWaitTime = timer.ElapsedNanoseconds();
  timer.Stop();

  // Compute the best pose (in the world coordinate frame)
  const bool foundEstimate = scoreMax > ipRegisters.mScoreMin;
  const Eigen::Vector3f bestMapPose =
    mapPoseMin.array() + step.array() * bestEstimate.cast<float>().array();
  const Eigen::Vector3f bestWorldPose =
    gridMapUtil.getWorldCoordsPose(bestMapPose);

  // Compute the pose covariance only if necessary
  if (pCovMatrix != nullptr) {
    // Compute the covariance matrix (in the world coordinate frame)
    const Eigen::Matrix3f covMatrixMap =
      gridMapUtil.getCovarianceForPose(bestMapPose, dataContainer);
    const Eigen::Matrix3f covMatrixWorld =
      gridMapUtil.getCovMatrixWorldCoords(covMatrixMap);
    // Set the covariance matrix (in the map coordinate frame)
    *pCovMatrix = covMatrixMap;
  }

  // Fill the metrics information if necessary
  if (pMetrics != nullptr) {
    const Eigen::Vector3f poseDiffWorld = bestWorldPose - initialWorldPose;
    const float normalizedScore = static_cast<float>(scoreMax)
      / static_cast<float>((1 << this->mIPConfig.mBitsPerValue) - 1)
      / static_cast<float>(ipRegisters.mNumOfScans);
    pMetrics->optimization_time.fromNSec(outerTimer.ElapsedNanoseconds());
    pMetrics->input_setup_time.fromNSec(inputSetupTime);
    pMetrics->ip_setup_time.fromNSec(ipSetupTime);
    pMetrics->scan_transfer_time.fromNSec(scanTransferTime);
    pMetrics->map_transfer_time.fromNSec(mapTransferTime);
    pMetrics->ip_optimization_time.fromNSec(ipOptimizationTime);
    pMetrics->ip_wait_time.fromNSec(ipWaitTime);
    pMetrics->diff_translation = poseDiffWorld.head<2>().norm();
    pMetrics->diff_rotation = util::NormalizeAngleDifference(poseDiffWorld[2]);
    pMetrics->win_size_x = halfWindowSize[0] * 2 + 1;
    pMetrics->win_size_y = halfWindowSize[1] * 2 + 1;
    pMetrics->win_size_theta = halfWindowSize[2] * 2 + 1;
    pMetrics->step_size_x = gridMapUtil.getCellLength();
    pMetrics->step_size_y = gridMapUtil.getCellLength();
    pMetrics->step_size_theta = stepTheta;
    pMetrics->transferred_map_size_x = transferredMapSize.x();
    pMetrics->transferred_map_size_y = transferredMapSize.y();
    pMetrics->num_of_transferred_map_blocks = numOfTransferredNumBlocks;
    pMetrics->score = normalizedScore;
    pMetrics->num_of_scans = ipRegisters.mNumOfScans;
  }

  return bestWorldPose;
}

// Initialize the contiguous memory to store the inputs
void ScanMatcherFPGA::InitializeCMAForInput()
{
  // Compute the number of bytes required to transfer the scan
  // Each 64-bit element packs the scan range (32-bit floating point) and
  // the scan angle (32-bit floating point)
  // Add 1 to transfer the flag which indicates whether the scan is transferred
  const std::size_t scanDataLength = this->mIPConfig.mNumOfScansMax + 1;

  // Compute the number of bytes required to transfer the grid map
  // Each 64-bit element (grid map chunk) packs 8 occupancy probability
  // values for 8 consecutive grid cells, which are quantized to
  // 8-bit unsigned integers when transferred
  const std::size_t mapChunksPerRow =
    (this->mIPConfig.mMapSizeXMax + this->mIPConfig.mGridCellsPerChunk - 1)
    / this->mIPConfig.mGridCellsPerChunk;
  // Add 1 to transfer the flag that indicates whether the grid map is
  // transferred (grid map is cached on the BRAM to reduce the expensive
  // data transfer cost)
  const std::size_t gridMapLength =
    this->mIPConfig.mMapSizeYMax * mapChunksPerRow + 1;

  // Compute the number of bytes required to store the input data
  const std::uint32_t lengthInBytes = static_cast<std::uint32_t>(
    std::max(scanDataLength, gridMapLength) * sizeof(std::uint64_t));
  // Allocate the contiguous memory for the input data
  this->mInputData.Initialize(lengthInBytes);
}

// Initialize the contiguous memory to store the outputs
void ScanMatcherFPGA::InitializeCMAForOutput()
{
  // Compute the number of bytes required to store the output data
  // The first 64-bit element contains the maximum score and the discretized
  // X-coordinate of the final sensor pose, and the second 64-bit element
  // contains the discretized Y-coordinate and the discretized rotation angle
  // of the final sensor pose in the sensor coordinate frame
  const std::uint32_t lengthInBytes = static_cast<std::uint32_t>(
    2 * sizeof(std::uint64_t));
  // Allocate the contiguous memory for the output data
  this->mOutputData.Initialize(lengthInBytes);
}

// Set the control registers of the scan matcher IP core
void ScanMatcherFPGA::SetIPRegisters(
  const ScanMatcherIPRegisters& ipRegisters)
{
  // Write the number of the scan points
  this->mIPRegisters->Write(this->mIPConfig.mAxiLiteSNumOfScans,
    I32ToU32(ipRegisters.mNumOfScans));
  // Write the maximum valid scan range
  this->mIPRegisters->Write(this->mIPConfig.mAxiLiteSScanRangeMax,
    FloatToU32(ipRegisters.mValidScanRangeMax));
  // Write the minimum score to accept the estimated transformation
  this->mIPRegisters->Write(this->mIPConfig.mAxiLiteSScoreThreshold,
    I32ToU32(ipRegisters.mScoreMin));

  // Write the minimum pose (in the scaled map coordinate frame)
  this->mIPRegisters->Write(this->mIPConfig.mAxiLiteSPoseX,
    FloatToU32(ipRegisters.mPoseMin[0]));
  this->mIPRegisters->Write(this->mIPConfig.mAxiLiteSPoseY,
    FloatToU32(ipRegisters.mPoseMin[1]));
  this->mIPRegisters->Write(this->mIPConfig.mAxiLiteSPoseTheta,
    FloatToU32(ipRegisters.mPoseMin[2]));
  // Write the size of the grid map to transfer
  this->mIPRegisters->Write(this->mIPConfig.mAxiLiteSMapSizeX,
    I32ToU32(ipRegisters.mMapSize.x()));
  this->mIPRegisters->Write(this->mIPConfig.mAxiLiteSMapSizeY,
    I32ToU32(ipRegisters.mMapSize.y()));
  // Write the minimum position of the grid map to transfer
  this->mIPRegisters->Write(this->mIPConfig.mAxiLiteSMapMinX,
    FloatToU32(ipRegisters.mMapPosMin.x()));
  this->mIPRegisters->Write(this->mIPConfig.mAxiLiteSMapMinY,
    FloatToU32(ipRegisters.mMapPosMin.y()));

  // Write the size of the searching window
  this->mIPRegisters->Write(this->mIPConfig.mAxiLiteSWinX,
    I32ToU32(ipRegisters.mWindowSize[0]));
  this->mIPRegisters->Write(this->mIPConfig.mAxiLiteSWinY,
    I32ToU32(ipRegisters.mWindowSize[1]));
  this->mIPRegisters->Write(this->mIPConfig.mAxiLiteSWinTheta,
    I32ToU32(ipRegisters.mWindowSize[2]));
  // Write the search step (in the scaled map coordinate frame)
  this->mIPRegisters->Write(this->mIPConfig.mAxiLiteSStepX,
    FloatToU32(ipRegisters.mSearchStep[0]));
  this->mIPRegisters->Write(this->mIPConfig.mAxiLiteSStepY,
    FloatToU32(ipRegisters.mSearchStep[1]));
  this->mIPRegisters->Write(this->mIPConfig.mAxiLiteSStepTheta,
    FloatToU32(ipRegisters.mSearchStep[2]));
}

// Start the scan matcher IP core
void ScanMatcherFPGA::StartIP()
{
  // Set the control register to start the IP core
  this->mIPRegisters->Write(this->mIPConfig.mAxiLiteSApCtrl,
    static_cast<std::uint32_t>(hw::AxiLiteSApCtrl::Start));
  // Wait until the IP core starts by polling the control register
  while (!(this->mIPRegisters->Read(this->mIPConfig.mAxiLiteSApCtrl) &
           static_cast<std::uint32_t>(hw::AxiLiteSApCtrl::Start)));
}

// Wait until the scan matcher IP core is in idle state
void ScanMatcherFPGA::WaitIP()
{
  // Wait until the IP core is in idle state by polling the control register
  while (!(this->mIPRegisters->Read(this->mIPConfig.mAxiLiteSApCtrl) &
           static_cast<std::uint32_t>(hw::AxiLiteSApCtrl::Idle)));
}

// Transfer the scan through AXI4-Stream interface
void ScanMatcherFPGA::TransferScan(
  const DataContainer& dataContainer,
  const int numOfScans, const float scale)
{
  // Get the pointer to the contiguous memory
  auto* pInput = this->mInputData.Ptr<volatile std::uint64_t>();
  // Write the flag to indicate that the scan is transferred
  *pInput++ = static_cast<std::uint64_t>(1);

  const auto pointToPacket = [scale](const Eigen::Vector2f& point) {
    // Scale the range since points in `dataContainer` are represented in
    // the map coordinate frame
    const float range = std::hypot(point.x(), point.y()) * scale;
    const float angle = std::atan2(point.y(), point.x());
    return PackFloat(range, angle); };

  // Subsample the scan if the number of scan points exceeds the maximum
  if (numOfScans < dataContainer.getSize()) {
    const float interval = static_cast<float>(numOfScans - 1)
      / static_cast<float>(dataContainer.getSize() - 1);
    float sampledIdx = 0.0f;

    for (int i = 0; i < numOfScans; ++i) {
      const int idx = static_cast<int>(sampledIdx);
      *pInput++ = pointToPacket(dataContainer.getVecEntry(idx));
      sampledIdx += interval;
    }
  } else {
    for (int i = 0; i < numOfScans; ++i)
      *pInput++ = pointToPacket(dataContainer.getVecEntry(i));
  }

  // Compute the number of bytes to transfer
  const std::size_t bytesLength = (numOfScans + 1) * sizeof(std::uint64_t);
  this->mAxiDma->SendChannel().Transfer(
    bytesLength, this->mInputData.PhysicalAddress());
  // Wait for the data transfer completion
  this->mAxiDma->SendChannel().Wait();
}

// Transfer the grid map through AXI4-Stream interface
void ScanMatcherFPGA::TransferMap(
  const OccGridMapUtilConfig<GridMap>& gridMapUtil,
  const Eigen::AlignedBox2i& boundingBox,
  Eigen::Vector2i& transferredMapSize,
  int& numOfTransferredMapBlocks)
{
  // Get the pointer to the contiguous memory
  auto* pInput = this->mInputData.Ptr<volatile std::uint64_t>();
  // Write the flag to indicate that the scan is transferred
  *pInput++ = static_cast<std::uint64_t>(1);

  const int mapSizeX = gridMapUtil.getMapSizeX();
  const int mapSizeY = gridMapUtil.getMapSizeY();

  // Adjust the bounding box
  // `gridCellsPerChunk` is the number of grid cells in the 64-bit packet
  // (AXI DMA controller transfers 64-bit data per clock)
  const int gridCellsPerChunk = this->mIPConfig.mGridCellsPerChunk;
  const int chunksPerRow = (boundingBox.sizes().x() + gridCellsPerChunk - 1)
    / gridCellsPerChunk;

  // Copy the grid map values to the contiguous memory
  for (int y = 0; y < boundingBox.sizes().y(); ++y) {
    for (int x = 0, cellX = 0; x < chunksPerRow; ++x) {
      // Create a 64-bit packet (chunk) from the consecutive grid cells
      U64Conversion chunk;

      for (int c = 0; c < gridCellsPerChunk; ++c, ++cellX) {
        const Eigen::Vector2i idx {
          boundingBox.min().x() + cellX, boundingBox.min().y() + y };
        const float probability = std::min(1.0f, std::max(0.0f,
          gridMapUtil.getUnfilteredGridPoint(idx.y() * mapSizeX + idx.x())));
        chunk.mU8Values[c] = static_cast<std::uint8_t>(probability * 255.0f);
      }

      *pInput++ = chunk.mU64Value;
    }
  }

  // Compute the number of bytes to transfer
  const std::size_t bytesLength = (boundingBox.sizes().y() * chunksPerRow + 1)
    * sizeof(std::uint64_t);
  this->mAxiDma->SendChannel().Transfer(
    bytesLength, this->mInputData.PhysicalAddress());
  // Wait for the data transfer completion
  this->mAxiDma->SendChannel().Wait();

  // Set the size of the transferred grid map
  transferredMapSize.x() = chunksPerRow * gridCellsPerChunk;
  transferredMapSize.y() = boundingBox.sizes().y();
  // Set the number of the transferred blocks
  numOfTransferredMapBlocks = boundingBox.sizes().y() * chunksPerRow;
}

// Receive the result from AXI4-Stream interface
void ScanMatcherFPGA::ReceiveResult(
  Eigen::Vector3i& bestEstimate, int& scoreMax)
{
  // Compute the number of bytes to receive
  const std::size_t bytesLength = 2 * sizeof(std::uint64_t);
  this->mAxiDma->RecvChannel().Transfer(
    bytesLength, this->mOutputData.PhysicalAddress());
  // Wait for the data transfer completion
  this->mAxiDma->RecvChannel().Wait();

  // Get the pointer to the contiguous memory
  const auto* pOutput = this->mOutputData.Ptr<volatile std::uint64_t>();
  // Read the maximum score and the X-coordinate of the best estimate
  UnpackI32(*pOutput++, scoreMax, bestEstimate[0]);
  // Read the Y-coordinate and angle of the best estimate
  UnpackI32(*pOutput++, bestEstimate[1], bestEstimate[2]);
}

// Compute the bounding box of the grid map to transfer
Eigen::AlignedBox2i ScanMatcherFPGA::ComputeBoundingBox(
  const OccGridMapUtilConfig<GridMap>& gridMapUtil,
  const Eigen::Vector3f& centerMapPose)
{
  const int mapSizeXMax = this->mIPConfig.mMapSizeXMax;
  const int mapSizeYMax = this->mIPConfig.mMapSizeYMax;

  // Compute the center position of the grid map to transfer
  // Try to transfer the grid cells as much as possible
  const Eigen::Vector2i desiredCenterIdx = centerMapPose.head<2>().cast<int>();
  const Eigen::Vector2i idxMin {
    std::min(gridMapUtil.getMapSizeX() - 1,
      std::max(0, desiredCenterIdx.x() - mapSizeXMax)),
    std::min(gridMapUtil.getMapSizeY() - 1,
      std::max(0, desiredCenterIdx.y() - mapSizeYMax)) };
  const Eigen::Vector2i idxMax {
    std::min(gridMapUtil.getMapSizeX(),
      std::max(1, desiredCenterIdx.x() + mapSizeXMax)),
    std::min(gridMapUtil.getMapSizeY(),
      std::max(1, desiredCenterIdx.y() + mapSizeYMax)) };
  const Eigen::Vector2i centerIdx = (idxMin + idxMax) / 2;

  // Compute the bounding box
  const Eigen::Vector2i finalIdxMin {
    std::max(0, centerIdx.x() - mapSizeXMax / 2),
    std::max(0, centerIdx.y() - mapSizeYMax / 2) };
  const Eigen::Vector2i finalIdxMax {
    std::min(gridMapUtil.getMapSizeX(), centerIdx.x() + mapSizeXMax / 2),
    std::min(gridMapUtil.getMapSizeY(), centerIdx.y() + mapSizeYMax / 2) };
  return Eigen::AlignedBox2i { finalIdxMin, finalIdxMax };
}

} // namespace hectorslam
