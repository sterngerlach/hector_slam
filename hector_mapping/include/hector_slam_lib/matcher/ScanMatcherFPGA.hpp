
// ScanMatcherFPGA.hpp

#ifndef HECTOR_SLAM_MATCHER_SCAN_MATCHER_FPGA_HPP
#define HECTOR_SLAM_MATCHER_SCAN_MATCHER_FPGA_HPP

#include <cstdint>
#include <memory>
#include <Eigen/Core>

#include "hw/ap_ctrl.hpp"
#include "hw/axi_simple_dma.hpp"
#include "hw/cma_memory.hpp"
#include "hw/mmio.hpp"
#include "map/GridMap.h"
#include "map/OccGridMapUtilConfig.h"
#include "scan/DataPointContainer.h"
#include "util/DrawInterface.h"
#include "util/HectorDebugInfoInterface.h"

namespace hectorslam {

//
// ScanMatcherIPConfig struct holds the necessary information for the
// real-time correlative scan matcher IP core, including the scan matching
// parameters, the base address and address range of the AXI4-Lite interface,
// the address offsets of the registers
//
struct ScanMatcherIPConfig
{
  // Maximum number of the scan points
  int mNumOfScansMax;
  // Maximum width of the grid map (in the number of grid cells)
  int mMapSizeXMax;
  // Maximum height of the grid map (in the number of grid cells)
  int mMapSizeYMax;
  // Resolution of the coarse grid map (in the number of grid cells)
  int mCoarseMapResolution;
  // Bit width of the occupancy probability value
  int mBitsPerValue;
  // Width of the grid map chunk (consecutive grid map cells)
  int mGridCellsPerChunk;

  // Register offsets for the AXI4-Lite slave interface

  // AXI4-Lite slave interface base address
  std::uint32_t mAxiLiteSBaseAddress;
  // AXI4-Lite slave interface address range
  std::uint32_t mAxiLiteSAddressRange;
  // AXI4-Lite slave interface control signal
  std::uint32_t mAxiLiteSApCtrl;
  // AXI4-Lite slave interface global interrupt enable register
  std::uint32_t mAxiLiteSGIE;
  // AXI4-Lite slave interface IP interrupt enable register
  std::uint32_t mAxiLiteSIER;
  // AXI4-Lite slave interface IP interrupt status register
  std::uint32_t mAxiLiteSISR;

  // Register offsets for the scan matcher

  // Register offset for the actual number of the scan points
  std::uint32_t mAxiLiteSNumOfScans;
  // Register offset for the maximum scan range considered valid
  std::uint32_t mAxiLiteSScanRangeMax;
  // Register offset for the score threshold (for loop detection)
  std::uint32_t mAxiLiteSScoreThreshold;
  // Register offset for the sensor pose
  std::uint32_t mAxiLiteSPoseX;
  std::uint32_t mAxiLiteSPoseY;
  std::uint32_t mAxiLiteSPoseTheta;
  // Register offset for the actual size of the grid map
  std::uint32_t mAxiLiteSMapSizeX;
  std::uint32_t mAxiLiteSMapSizeY;
  // Register offset for the minimum coordinate of the grid map
  std::uint32_t mAxiLiteSMapMinX;
  std::uint32_t mAxiLiteSMapMinY;
  // Register offset for the size of the search window
  std::uint32_t mAxiLiteSWinX;
  std::uint32_t mAxiLiteSWinY;
  std::uint32_t mAxiLiteSWinTheta;
  // Register offset for the search step
  std::uint32_t mAxiLiteSStepX;
  std::uint32_t mAxiLiteSStepY;
  std::uint32_t mAxiLiteSStepTheta;
};

//
// AxiDmaConfig struct holds the necessary information for the AXI DMA IP core,
// especially the base address and the address range
// 
struct AxiDmaConfig
{
  // AXI DMA base address
  std::uint32_t mBaseAddress;
  // AXI DMA address range
  std::uint32_t mAddressRange;
};

//
// ScanMatcherIPRegisters struct holds the control registers which are written
// through the AXI4-Lite slave interface
//
struct ScanMatcherIPRegisters
{
  // Number of the scan points
  int mNumOfScans;
  // Maximum valid scan range
  float mValidScanRangeMax;
  // Minimum score to accept the estimated transformation
  int mScoreMin;
  // Minimum pose (in the scaled map coordinate frame)
  Eigen::Vector3f mPoseMin;
  // Size of the grid map to transfer
  Eigen::Vector2i mMapSize;
  // Minimum position of the grid map to transfer
  Eigen::Vector2f mMapPosMin;
  // Size of the searching window
  Eigen::Vector3i mWindowSize;
  // Search step (in the scaled map coordinate frame)
  Eigen::Vector3f mSearchStep;
};

//
// ScanMatcherFPGA class performs the real-time correlative scan matching
// on the dedicated IP core implemented on Zynq devices
//
class ScanMatcherFPGA final
{
public:
  // Constructor
  ScanMatcherFPGA(const int coarseMapResolution,
                  const Eigen::Vector3f& searchWindow,
                  const float angleSearchStepMin,
                  const float angleSearchStepMax,
                  const ScanMatcherIPConfig& ipConfig,
                  const AxiDmaConfig& axiDmaConfig,
                  DrawInterface* pDrawInterface = nullptr,
                  HectorDebugInfoInterface* pDebugInterface = nullptr);

  // Destructor
  ~ScanMatcherFPGA() = default;

  // Match scan to grid map given an initial world pose estimate
  Eigen::Vector3f MatchScans(
    const Eigen::Vector3f& initialWorldPose,
    OccGridMapUtilConfig<GridMap>& gridMapUtil,
    const DataContainer& dataContainer,
    Eigen::Matrix3f& covMatrix,
    const float scoreMin = 0.0f,
    const float correspondenceRatioMin = 0.0f);

private:
  // Initialize the contiguous memory to store the inputs
  void InitializeCMAForInput();
  // Initialize the contiguous memory to store the outputs
  void InitializeCMAForOutput();
  // Set the control registers of the scan matcher IP core
  void SetIPRegisters(const ScanMatcherIPRegisters& ipRegisters);
  // Start the scan matcher IP core
  void StartIP();
  // Wait until the scan matcher IP core is in idle state
  void WaitIP();
  // Transfer the scan through AXI4-Stream interface
  void TransferScan(const DataContainer& dataContainer,
                    const int numOfScans, const float scale);
  // Transfer the grid map through AXI4-Stream interface
  void TransferMap(OccGridMapUtilConfig<GridMap>& gridMapUtil,
                   const Eigen::AlignedBox2i& boundingBox);
  // Receive the result from AXI4-Stream interface
  void ReceiveResult(Eigen::Vector3i& bestEstimate, int& scoreMax);

  // Compute the bounding box of the grid map to transfer
  Eigen::AlignedBox2i ComputeBoundingBox(
    OccGridMapUtilConfig<GridMap>& gridMapUtil,
    const Eigen::Vector3f& centerMapPose);

private:
  // Resolution of a coarse grid map (in the number of grid cells)
  int mCoarseMapResolution;
  // Search window size
  Eigen::Vector3f mSearchWindow;
  // Minimum angle search step (in radians)
  float mAngleSearchStepMin;
  // Maximum angle search step (in radians)
  float mAngleSearchStepMax;
  // Configuration of the scan matcher IP core
  ScanMatcherIPConfig mIPConfig;
  // Configuration of the AXI DMA IP core
  AxiDmaConfig mAxiDmaConfig;
  // Memory-mapped I/O for the scan matcher IP core control registers
  std::unique_ptr<hw::MemoryMappedIO> mIPRegisters;
  // AXI DMA IP core
  std::unique_ptr<hw::AxiSimpleDMA> mAxiDma;
  // Contiguous memory to store the scan matching inputs
  hw::CMAMemory mInputData;
  // Contiguous memory to store the scan matching outputs
  hw::CMAMemory mOutputData;
  // Draw interface (for visualization in RViz)
  DrawInterface* mDrawInterface;
  // Debug information interface (for inspecting covariance matrices)
  HectorDebugInfoInterface* mDebugInterface;
};

} // namespace hectorslam

#endif // HECTOR_SLAM_MATCHER_SCAN_MATCHER_FPGA_HPP
