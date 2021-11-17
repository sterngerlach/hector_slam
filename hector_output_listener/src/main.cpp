
// main.cpp

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdlib>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>

#include <boost/version.hpp>

// Definitions to prevent compile errors
// `int_p_NULL` is removed in libpng 1.4
#define png_infopp_NULL     (png_infopp)NULL
#define int_p_NULL          (int*)NULL
#define png_bytep_NULL      (png_bytep)NULL

#if BOOST_VERSION <= 106700
#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_io.hpp>
#elif BOOST_VERSION <= 106800
#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png.hpp>
#else
#include <boost/gil.hpp>
#include <boost/gil/extension/io/png.hpp>
#endif

#include <boost/array.hpp>
#include <boost/filesystem.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "hector_mapping/GetOccupancyGrids.h"
#include "hector_mapping/HectorMappingMetrics.h"
#include "hector_mapping/ScanMatcherCorrelativeMetrics.h"
#include "hector_mapping/ScanMatcherFPGAMetrics.h"
#include "hector_mapping/ScanMatcherGaussNewtonMetrics.h"
#include "output_listener/SaveOutput.h"

#include "hector_output_listener/bresenham.hpp"
#include "hector_output_listener/util.hpp"

namespace fs = boost::filesystem;
namespace gil = boost::gil;
namespace pt = boost::property_tree;

using PoseStamped = geometry_msgs::PoseStamped;
using PoseStampedPtr = PoseStamped::ConstPtr;
using PoseWithCovarianceStamped = geometry_msgs::PoseWithCovarianceStamped;
using PoseWithCovarianceStampedPtr = PoseWithCovarianceStamped::ConstPtr;

using namespace hector_mapping;
using namespace output_listener;

// Convert scan matching option constant to const char* string
const char* ScanMatcherOptionToString(const int option)
{
  switch (option) {
    case HectorMappingMetrics::SCAN_MATCHER_UNKNOWN:
      return "Unknown";
    case HectorMappingMetrics::SCAN_MATCHER_DEFAULT:
      return "Default";
    case HectorMappingMetrics::SCAN_MATCHER_GAUSS_NEWTON:
      return "GaussNewton";
    case HectorMappingMetrics::SCAN_MATCHER_CORRELATIVE:
      return "Correlative";
    case HectorMappingMetrics::SCAN_MATCHER_CORRELATIVE_FPGA:
      return "CorrelativeFPGA";
    case HectorMappingMetrics::SCAN_MATCHER_GAUSS_NEWTON_AFTER_CORRELATIVE:
      return "GaussNewtonAfterCorrelative";
    case HectorMappingMetrics::SCAN_MATCHER_GAUSS_NEWTON_AFTER_CORRELATIVE_FPGA:
      return "GaussNewtonAfterCorrelativeFPGA";
  }

  return "Unknown";
}

struct NodeSettings
{
  // Flag to enable verbose outputs to the console
  bool mVerbose;
  // Flag to crop the occupancy grid map
  bool mCropGridMap;
  // Background color (unknown occupancy probability)
  std::array<std::uint8_t, 3> mBackgroundColor;
  // Trajectory line color
  std::array<std::uint8_t, 3> mTrajectoryLineColor;
  // Trajectory line width (in pixels)
  int mTrajectoryLineWidth;
};

class OutputListenerNode final
{
public:
  // Constructor
  OutputListenerNode() = default;
  // Destructor
  ~OutputListenerNode() = default;

  // Setup the node
  bool Setup(ros::NodeHandle& node);

  // Copy constructor (disabled)
  OutputListenerNode(const OutputListenerNode&) = delete;
  // Copy assignment operator (disabled)
  OutputListenerNode& operator=(const OutputListenerNode&) = delete;
  // Move constructor (disabled)
  OutputListenerNode(OutputListenerNode&&) = delete;
  // Move assignment operator (disabled)
  OutputListenerNode& operator=(OutputListenerNode&&) = delete;

  // Handler for the metrics message
  void OnHectorMappingMetrics(const HectorMappingMetrics::ConstPtr& metrics);
  // Handler for the pose message
  void OnPose(const PoseStampedPtr& pose);
  // Handler for the pose with covariance message
  void OnPoseWithCovariance(const PoseWithCovarianceStampedPtr& poseWithCov);

  // Save the metrics and grid map
  bool SaveOutput(SaveOutput::Request& request,
                  SaveOutput::Response& response);

private:
  // Save the metrics as JSON
  bool SaveMetrics(const std::string& fileName) const;
  // Save the grid map as PNG
  bool SaveMap(const std::string& baseFileName,
               const nav_msgs::OccupancyGrid& occupancyGrid,
               const std::vector<Eigen::Vector2f>& trajectory,
               const int resolutionLevel) const;
  // Save the grid map metadata as JSON
  void SaveMapMetadata(const std::string& fileName,
                       const nav_msgs::OccupancyGrid& occupancyGrid,
                       const Eigen::AlignedBox2i& boundingBox,
                       const int resolutionLevel) const;

  // Convert the metrics for Hector SLAM to the JSON tree
  pt::ptree ToJson(const HectorMappingMetrics::ConstPtr& metrics) const;
  // Convert the metrics for correlative scan matching to the JSON tree
  pt::ptree ToJson(const ScanMatcherCorrelativeMetrics& metrics) const;
  // Convert the metrics for FPGA-based scan matching to the JSON tree
  pt::ptree ToJson(const ScanMatcherFPGAMetrics& metrics) const;
  // Convert the metrics for Gauss-Newton scan matching to the JSON tree
  pt::ptree ToJson(const ScanMatcherGaussNewtonMetrics& metrics) const;

  // Compute the size of the cropped occupancy grid
  Eigen::AlignedBox2i GetBoundingBox(
    const nav_msgs::OccupancyGrid& occupancyGrid) const;
  // Draw the grid map
  bool DrawMap(const gil::rgb8_view_t& image,
               const nav_msgs::OccupancyGrid& occupancyGrid,
               const Eigen::AlignedBox2i& boundingBox) const;
  // Draw the trajectory
  bool DrawTrajectory(const gil::rgb8_view_t& image,
                      const std::vector<Eigen::Vector2f>& trajectory,
                      const Eigen::Vector2f& origin,
                      const float resolution) const;

private:
  // Subscriber for the metrics
  ros::Subscriber mSubMetrics;
  // Subscriber for the current pose (for testing)
  ros::Subscriber mSubPose;
  // Subscriber for the current pose with the covariance (for testing)
  ros::Subscriber mSubPoseWithCovariance;
  // Service client for receiving the grid map
  ros::ServiceClient mSrvGridMap;
  // Service server for saving the outputs
  ros::ServiceServer mSrvSaveOutputs;
  // Vector of the metrics
  std::vector<HectorMappingMetrics::ConstPtr> mMetrics;
  // Node parameter settings
  NodeSettings mSettings;
};

// Setup the node
bool OutputListenerNode::Setup(ros::NodeHandle& node)
{
  auto readColor = [](ros::NodeHandle& nodeHandle,
                      const std::string& paramName,
                      const std::string& defaultColor,
                      std::array<std::uint8_t, 3>& color) {
    std::string paramValue;
    nodeHandle.param(paramName, paramValue, defaultColor);

    // Parameter should be in the `#RRGGBB` format
    if (paramValue.size() != 7 || paramValue[0] != '#')
      return false;
    if (!std::all_of(paramValue.begin() + 1, paramValue.end(),
                     [](unsigned char c) { return std::isxdigit(c); }))
      return false;

    color[0] = static_cast<std::uint8_t>(
      std::stoul(paramValue.substr(1, 2), nullptr, 16));
    color[1] = static_cast<std::uint8_t>(
      std::stoul(paramValue.substr(3, 2), nullptr, 16));
    color[2] = static_cast<std::uint8_t>(
      std::stoul(paramValue.substr(5, 2), nullptr, 16));

    return true;
  };

  // Read parameter settings
  ros::NodeHandle privateNode { "~" };

  privateNode.param("verbose", this->mSettings.mVerbose, false);
  privateNode.param("crop_grid_map", this->mSettings.mCropGridMap, true);
  privateNode.param("trajectory_line_width",
                    this->mSettings.mTrajectoryLineWidth, 2);

  if (!readColor(privateNode, "background_color", "#C0C0C0",
                 this->mSettings.mBackgroundColor)) {
    ROS_ERROR("OutputListenerNode: Failed to read `background_color`");
    return false;
  }

  if (!readColor(privateNode, "trajectory_line_color", "#FF0000",
                 this->mSettings.mTrajectoryLineColor)) {
    ROS_ERROR("OutputListenerNode: Failed to read `trajectory_line_color`");
    return false;
  }

  this->mSubMetrics = node.subscribe<HectorMappingMetrics>(
    "metrics", 10, &OutputListenerNode::OnHectorMappingMetrics, this);
  this->mSubPose = node.subscribe<PoseStamped>(
    "slam_out_pose", 10, &OutputListenerNode::OnPose, this);
  this->mSubPoseWithCovariance = node.subscribe<PoseWithCovarianceStamped>(
    "poseupdate", 10, &OutputListenerNode::OnPoseWithCovariance, this);
  this->mSrvGridMap = node.serviceClient<GetOccupancyGrids>(
    "get_occupancy_grids", false);
  this->mSrvSaveOutputs = node.advertiseService(
    "save_output", &OutputListenerNode::SaveOutput, this);

  ROS_INFO("OutputListenerNode: Started");

  return true;
}

// Handler for the metrics message
void OutputListenerNode::OnHectorMappingMetrics(
  const HectorMappingMetrics::ConstPtr& metrics)
{
  // Append the metrics
  this->mMetrics.push_back(metrics);

  if (this->mSettings.mVerbose)
    ROS_INFO("OutputListenerNode: Received new metrics");
}

// Handler for the pose message
void OutputListenerNode::OnPose(const PoseStampedPtr& pose)
{
  const Eigen::Vector3d eulerAngles =
    ToEulerAngles(pose->pose.orientation);

  if (this->mSettings.mVerbose)
    ROS_INFO("OutputListenerNode: Received current pose: "
             "frame: %s, time: %f, x: %f, y: %f, yaw: %f",
             pose->header.frame_id.c_str(), pose->header.stamp.toSec(),
             pose->pose.position.x, pose->pose.position.y,
             eulerAngles.z());
}

// Handler for the pose with covariance message
void OutputListenerNode::OnPoseWithCovariance(
  const PoseWithCovarianceStampedPtr& poseWithCov)
{
  const Eigen::Vector3d eulerAngles =
    ToEulerAngles(poseWithCov->pose.pose.orientation);
  const Eigen::Matrix3d poseCovariance =
    ToPose2DCovariance(poseWithCov->pose.covariance.data());

  if (this->mSettings.mVerbose)
    ROS_INFO("OutputListenerNode: Received current pose with covariance: "
             "frame: %s, time: %f, x: %f, y: %f, yaw: %f, "
             "x-x: %f, x-y: %f, x-yaw: %f, "
             "y-y: %f, y-yaw: %f, yaw-yaw: %f",
             poseWithCov->header.frame_id.c_str(),
             poseWithCov->header.stamp.toSec(),
             poseWithCov->pose.pose.position.x,
             poseWithCov->pose.pose.position.y, eulerAngles.z(),
             poseCovariance(0, 0), poseCovariance(0, 1), poseCovariance(0, 2),
             poseCovariance(1, 1), poseCovariance(1, 2), poseCovariance(2, 2));
}

// Save the metrics and grid map
bool OutputListenerNode::SaveOutput(SaveOutput::Request& request,
                                    SaveOutput::Response& response)
{
  // Mark as the failure
  response.result = false;

  // Check that the parent directory exists
  const fs::path metricsFileDir =
    fs::path(request.metrics_file_name).parent_path();
  const fs::path gridMapFileDir =
    fs::path(request.map_base_file_name).parent_path();

  if (!fs::exists(metricsFileDir)) {
    ROS_ERROR("Directory %s does not exist", metricsFileDir.c_str());
    return true;
  }

  if (!fs::exists(gridMapFileDir)) {
    ROS_ERROR("Directory %s does not exist", gridMapFileDir.c_str());
    return true;
  }

  // Save the metrics as JSON
  if (!this->SaveMetrics(request.metrics_file_name)) {
    ROS_ERROR("Failed to save the metrics to %s",
              request.metrics_file_name.c_str());
    return true;
  }

  // Get the occupancy grids by calling the service
  GetOccupancyGrids srvOccupancyGrid;
  if (!this->mSrvGridMap.call(srvOccupancyGrid)) {
    ROS_ERROR("Failed to call the service %s",
              this->mSrvGridMap.getService().c_str());
    return true;
  }

  // Build the robot trajectory from the metrics
  std::vector<Eigen::Vector2f> trajectory;
  trajectory.reserve(this->mMetrics.size());
  float travelDistance = 0.0f;

  if (!this->mMetrics.empty())
    trajectory.emplace_back(this->mMetrics.front()->final_pose_estimate[0],
                            this->mMetrics.front()->final_pose_estimate[1]);

  for (std::size_t i = 1; i < this->mMetrics.size(); ++i) {
    Eigen::Vector2f pose { this->mMetrics[i]->final_pose_estimate[0],
                           this->mMetrics[i]->final_pose_estimate[1] };
    travelDistance += (pose - trajectory.back()).norm();

    // Only append if the robot has travelled a sufficient distance
    if (travelDistance >= 0.05f) {
      trajectory.push_back(std::move(pose));
      travelDistance = 0.0f;
    }
  }

  // Save the occupancy grid and metadata
  const auto& occupancyGrids = srvOccupancyGrid.response.occupancy_grids;
  const auto& resolutionLevels = srvOccupancyGrid.response.resolution_levels;

  if (occupancyGrids.size() != resolutionLevels.size()) {
    ROS_ERROR("Number of the occupancy grids is different from the number of "
              "the metadata returned from the Hector SLAM node");
    return true;
  }

  for (std::size_t i = 0; i < occupancyGrids.size(); ++i) {
    const auto baseFileName = request.map_base_file_name + '-'
                              + std::to_string(resolutionLevels[i]);
    if (!this->SaveMap(baseFileName, occupancyGrids[i],
                       trajectory, resolutionLevels[i])) {
      ROS_ERROR("Failed to save the occupancy grid (resolution level: %d) "
                "to the directory %s",
                resolutionLevels[i], gridMapFileDir.c_str());
      return true;
    }
  }

  // Mark as the successful
  response.result = true;
  return true;
}

// Save the metrics as JSON
bool OutputListenerNode::SaveMetrics(const std::string& fileName) const
{
  // Create JSON tree from the metrics
  pt::ptree jsonMetrics;

  for (std::size_t i = 0; i < this->mMetrics.size(); ++i)
    jsonMetrics.add_child(std::to_string(i), this->ToJson(this->mMetrics[i]));

  try {
    pt::write_json(fileName, jsonMetrics);
    ROS_INFO("Metrics are saved to %s", fileName.c_str());
  } catch (const pt::json_parser_error& e) {
    ROS_ERROR("Failed to save the metrics to %s, message: %s",
              fileName.c_str(), e.what());
    return false;
  }

  return true;
}

// Save the grid map as PNG
bool OutputListenerNode::SaveMap(
  const std::string& baseFileName,
  const nav_msgs::OccupancyGrid& occupancyGrid,
  const std::vector<Eigen::Vector2f>& trajectory,
  const int resolutionLevel) const
{
  // Compute the size of the cropped occupancy grid
  const Eigen::AlignedBox2i entireBox {
    Eigen::Vector2i(0, 0),
    Eigen::Vector2i(occupancyGrid.info.width,
                    occupancyGrid.info.height) };
  const Eigen::AlignedBox2i boundingBox = this->mSettings.mCropGridMap ?
    this->GetBoundingBox(occupancyGrid) : entireBox;

  // Initialize the image
  gil::rgb8_image_t mapImage { boundingBox.sizes().x(),
                               boundingBox.sizes().y() };
  const gil::rgb8_view_t& mapImageView = gil::view(mapImage);
  const gil::rgb8_pixel_t backgroundColor {
    this->mSettings.mBackgroundColor[0],
    this->mSettings.mBackgroundColor[1],
    this->mSettings.mBackgroundColor[2] };
  gil::fill_pixels(mapImageView, backgroundColor);

  // Draw the occupancy probabilities to the image
  if (!this->DrawMap(mapImageView, occupancyGrid, boundingBox)) {
    ROS_ERROR("Failed to create an image from the occupancy grid");
    return false;
  }

  // Draw the robot trajectory to the image
  const auto& mapOrigin = occupancyGrid.info.origin.position;
  const float resolution = occupancyGrid.info.resolution;
  const Eigen::Vector2f origin {
    mapOrigin.x + boundingBox.min().x() * resolution,
    mapOrigin.y + boundingBox.min().y() * resolution };
  if (!this->DrawTrajectory(mapImageView, trajectory, origin, resolution)) {
    ROS_ERROR("Failed to draw robot trajectory on the occupancy grid image");
    return false;
  }

  // Save the grid map as PNG
  const std::string fileName = baseFileName + ".png";
  const auto& flippedImage = gil::flipped_up_down_view(mapImageView);

  try {
#if BOOST_VERSION <= 106700
    gil::png_write_view(fileName, flippedImage);
#else
    gil::write_view(fileName, flippedImage, gil::png_tag());
#endif
    ROS_INFO("Occupancy grid image is saved to %s", fileName.c_str());
  } catch (const std::ios_base::failure& e) {
    ROS_ERROR("Failed to save the occupancy grid image to %s, message: %s",
              fileName.c_str(), e.what());
    return false;
  }

  // Save the grid map metadata as JSON
  const std::string metadataFileName = baseFileName + ".json";

  try {
    this->SaveMapMetadata(metadataFileName, occupancyGrid,
                          boundingBox, resolutionLevel);
    ROS_INFO("Metadata for the occupancy grid is saved to %s",
             metadataFileName.c_str());
  } catch (const pt::json_parser_error& e) {
    ROS_ERROR("Failed to save the metadata for the occupancy grid to %s, "
              "message: %s",
              metadataFileName.c_str(), e.what());
    return false;
  }

  return true;
}

// Save the grid map metadata as JSON
void OutputListenerNode::SaveMapMetadata(
  const std::string& fileName,
  const nav_msgs::OccupancyGrid& occupancyGrid,
  const Eigen::AlignedBox2i& boundingBox,
  const int resolutionLevel) const
{
  pt::ptree jsonMetadata;

  jsonMetadata.put("Stamp", occupancyGrid.header.stamp.toNSec());
  jsonMetadata.put("FrameId", occupancyGrid.header.frame_id);
  jsonMetadata.put("Seq", occupancyGrid.header.seq);

  const auto mapWidth = occupancyGrid.info.width;
  const auto mapHeight = occupancyGrid.info.height;
  const auto& mapOrigin = occupancyGrid.info.origin.position;
  const float resolution = occupancyGrid.info.resolution;
  const Eigen::Vector2f origin {
    mapOrigin.x + boundingBox.min().x() * resolution,
    mapOrigin.y + boundingBox.min().y() * resolution };

  jsonMetadata.put("MapOriginX", static_cast<float>(mapOrigin.x));
  jsonMetadata.put("MapOriginY", static_cast<float>(mapOrigin.y));
  jsonMetadata.put("MapSizeX", mapWidth);
  jsonMetadata.put("MapSizeY", mapHeight);
  jsonMetadata.put("ImageOriginX", origin.x());
  jsonMetadata.put("ImageOriginY", origin.y());
  jsonMetadata.put("ImageSizeX", boundingBox.sizes().x());
  jsonMetadata.put("ImageSizeY", boundingBox.sizes().y());
  jsonMetadata.put("Resolution", resolution);
  jsonMetadata.put("ResolutionLevel", resolutionLevel);

  // Save the grid map metadata as JSON
  pt::write_json(fileName, jsonMetadata);
}

// Convert the metrics for Hector SLAM to the JSON tree
pt::ptree OutputListenerNode::ToJson(
  const HectorMappingMetrics::ConstPtr& metrics) const
{
  pt::ptree jsonMetrics;

  jsonMetrics.put("Stamp", metrics->stamp);
  jsonMetrics.put("ScanTimeStamp", metrics->scan_time_stamp);

  jsonMetrics.put("ProcessingTime",
                  metrics->processing_time.toNSec());
  jsonMetrics.put("ScanPreprocessingTime",
                  metrics->scan_preprocessing_time.toNSec());
  jsonMetrics.put("ScanMatchingTime",
                  metrics->scan_matching_time.toNSec());
  jsonMetrics.put("MapUpdateTime",
                  metrics->map_update_time.toNSec());
  jsonMetrics.put("ResultPublishTime",
                  metrics->result_publish_time.toNSec());

  // Write the boolean flag (std::uint8_t) as integer, since std::uint8_t
  // values are printed out as control characters (NUL and SOH)
  jsonMetrics.put("ScanMatchingSkipped",
                  metrics->scan_matching_skipped ? 1 : 0);
  jsonMetrics.put("MapUpdateSkipped",
                  metrics->map_update_skipped ? 1 : 0);

  jsonMetrics.put("InitialPose",
                  Pose2DToString(metrics->initial_pose_estimate));
  jsonMetrics.put("FinalPose",
                  Pose2DToString(metrics->final_pose_estimate));
  jsonMetrics.put("PoseCovariance",
                  Pose2DCovarianceToString(metrics->pose_covariance));

  jsonMetrics.put("ScanMatcherOption",
                  ScanMatcherOptionToString(metrics->scan_matcher_option));

  pt::ptree jsonGaussNewtonMetrics;
  pt::ptree jsonCorrelativeMetrics;
  pt::ptree jsonFPGAMetrics;

  switch (metrics->scan_matcher_option) {
    case HectorMappingMetrics::SCAN_MATCHER_GAUSS_NEWTON:
      for (const auto& gaussNewtonMetric :
           metrics->scan_matcher_gauss_newton_metrics)
        jsonGaussNewtonMetrics.add_child("", this->ToJson(gaussNewtonMetric));
      break;

    case HectorMappingMetrics::SCAN_MATCHER_CORRELATIVE:
      for (const auto& correlativeMetric :
           metrics->scan_matcher_correlative_metrics)
        jsonCorrelativeMetrics.add_child("", this->ToJson(correlativeMetric));
      break;

    case HectorMappingMetrics::SCAN_MATCHER_CORRELATIVE_FPGA:
      for (const auto& fpgaMetric :
           metrics->scan_matcher_fpga_metrics)
        jsonFPGAMetrics.add_child("", this->ToJson(fpgaMetric));
      break;

    case HectorMappingMetrics::SCAN_MATCHER_GAUSS_NEWTON_AFTER_CORRELATIVE:
      for (const auto& correlativeMetric :
           metrics->scan_matcher_correlative_metrics)
        jsonCorrelativeMetrics.add_child("", this->ToJson(correlativeMetric));
      for (const auto& gaussNewtonMetric :
           metrics->scan_matcher_gauss_newton_metrics)
        jsonGaussNewtonMetrics.add_child("", this->ToJson(gaussNewtonMetric));
      break;

    case HectorMappingMetrics::SCAN_MATCHER_GAUSS_NEWTON_AFTER_CORRELATIVE_FPGA:
      for (const auto& fpgaMetric :
           metrics->scan_matcher_fpga_metrics)
        jsonFPGAMetrics.add_child("", this->ToJson(fpgaMetric));
      for (const auto& gaussNewtonMetric :
           metrics->scan_matcher_gauss_newton_metrics)
        jsonGaussNewtonMetrics.add_child("", this->ToJson(gaussNewtonMetric));
      break;
  }

  jsonMetrics.put_child("ScanMatcherGaussNewtonMetrics",
                        jsonGaussNewtonMetrics);
  jsonMetrics.put_child("ScanMatcherCorrelativeMetrics",
                        jsonCorrelativeMetrics);
  jsonMetrics.put_child("ScanMatcherFPGAMetrics",
                        jsonFPGAMetrics);

  return jsonMetrics;
}

// Convert the metrics for correlative scan matching to the JSON tree
pt::ptree OutputListenerNode::ToJson(
  const ScanMatcherCorrelativeMetrics& metrics) const
{
  pt::ptree jsonMetrics;

  jsonMetrics.put("Stamp", metrics.stamp);
  jsonMetrics.put("MapResolutionLevel", metrics.map_resolution_level);

  jsonMetrics.put("OptimizationTime", metrics.optimization_time.toNSec());
  jsonMetrics.put("DiffTranslation", metrics.diff_translation);
  jsonMetrics.put("DiffRotation", metrics.diff_rotation);

  const std::array<int, 3> winSize = {
    metrics.win_size_x, metrics.win_size_y, metrics.win_size_theta };
  const std::array<float, 3> stepSize = {
    metrics.step_size_x, metrics.step_size_y, metrics.step_size_theta };

  jsonMetrics.put("WindowSize", ArrayToString(
    winSize.data(), winSize.size()));
  jsonMetrics.put("StepSize", ArrayToString(
    stepSize.data(), stepSize.size(), std::numeric_limits<float>::digits10));
  jsonMetrics.put("NumOfSkippedNodes", metrics.num_of_skipped_nodes);
  jsonMetrics.put("NumOfProcessedNodes", metrics.num_of_processed_nodes);
  jsonMetrics.put("Score", metrics.score);
  jsonMetrics.put("NumOfScans", metrics.num_of_scans);

  return jsonMetrics;
}

// Convert the metrics for FPGA-based scan matching to the JSON tree
pt::ptree OutputListenerNode::ToJson(
  const ScanMatcherFPGAMetrics& metrics) const
{
  pt::ptree jsonMetrics;

  jsonMetrics.put("Stamp", metrics.stamp);
  jsonMetrics.put("MapResolutionLevel", metrics.map_resolution_level);

  jsonMetrics.put("OptimizationTime", metrics.optimization_time.toNSec());
  jsonMetrics.put("InputSetupTime", metrics.input_setup_time.toNSec());
  jsonMetrics.put("IPSetupTime", metrics.ip_setup_time.toNSec());
  jsonMetrics.put("ScanTransferTime", metrics.scan_transfer_time.toNSec());
  jsonMetrics.put("MapTransferTime", metrics.map_transfer_time.toNSec());
  jsonMetrics.put("IPOptimizationTime", metrics.ip_optimization_time.toNSec());
  jsonMetrics.put("IPWaitTime", metrics.ip_wait_time.toNSec());

  jsonMetrics.put("DiffTranslation", metrics.diff_translation);
  jsonMetrics.put("DiffRotation", metrics.diff_rotation);

  const std::array<int, 3> winSize = {
    metrics.win_size_x, metrics.win_size_y, metrics.win_size_theta };
  const std::array<float, 3> stepSize = {
    metrics.step_size_x, metrics.step_size_y, metrics.step_size_theta };
  const std::array<int, 2> transferredMapSize = {
    metrics.transferred_map_size_x, metrics.transferred_map_size_y };

  jsonMetrics.put("WindowSize", ArrayToString(
    winSize.data(), winSize.size()));
  jsonMetrics.put("StepSize", ArrayToString(
    stepSize.data(), stepSize.size(), std::numeric_limits<float>::digits10));
  jsonMetrics.put("TransferredMapSize", ArrayToString(
    transferredMapSize.data(), transferredMapSize.size()));
  jsonMetrics.put("NumOfTransferredMapBlocks",
                  metrics.num_of_transferred_map_blocks);

  jsonMetrics.put("Score", metrics.score);
  jsonMetrics.put("NumOfScans", metrics.num_of_scans);

  return jsonMetrics;
}

// Convert the metrics for Gauss-Newton scan matching to the JSON tree
pt::ptree OutputListenerNode::ToJson(
  const ScanMatcherGaussNewtonMetrics& metrics) const
{
  pt::ptree jsonMetrics;

  jsonMetrics.put("Stamp", metrics.stamp);
  jsonMetrics.put("MapResolutionLevel", metrics.map_resolution_level);

  jsonMetrics.put("OptimizationTime", metrics.optimization_time.toNSec());
  jsonMetrics.put("DiffTranslation", metrics.diff_translation);
  jsonMetrics.put("DiffRotation", metrics.diff_rotation);
  jsonMetrics.put("NumOfIterations", metrics.num_of_iterations);
  jsonMetrics.put("InitialCorrespondenceCost",
                  metrics.initial_correspondence_cost);
  jsonMetrics.put("FinalCorrespondenceCost",
                  metrics.final_correspondence_cost);
  jsonMetrics.put("NumOfScans", metrics.num_of_scans);

  return jsonMetrics;
}

// Compute the size of the cropped occupancy grid
Eigen::AlignedBox2i OutputListenerNode::GetBoundingBox(
  const nav_msgs::OccupancyGrid& occupancyGrid) const
{
  Eigen::Vector2i idxMin { std::numeric_limits<int>::max(),
                           std::numeric_limits<int>::max() };
  Eigen::Vector2i idxMax { std::numeric_limits<int>::min(),
                           std::numeric_limits<int>::min() };

  const int mapWidth = static_cast<int>(occupancyGrid.info.width);
  const int mapHeight = static_cast<int>(occupancyGrid.info.height);

  for (int row = 0; row < mapHeight; ++row) {
    for (int col = 0; col < mapWidth; ++col) {
      const int idx = row * mapWidth + col;

      // `-1` is the unknown occupancy probability
      if (occupancyGrid.data[idx] == -1)
        continue;

      idxMin.x() = std::min(idxMin.x(), col);
      idxMin.y() = std::min(idxMin.y(), row);
      idxMax.x() = std::max(idxMax.x(), col);
      idxMax.y() = std::max(idxMax.y(), row);
    }
  }

  // If the grid map has no grid cells with known occupancy probabilities,
  // then return the bounding box of the entire grid map
  if ((idxMin.array() > idxMax.array()).any())
    return Eigen::AlignedBox2i { Eigen::Vector2i(0, 0),
                                 Eigen::Vector2i(mapWidth, mapHeight) };

  return Eigen::AlignedBox2i { idxMin, idxMax.array() + 1 };
}

// Draw the grid map
bool OutputListenerNode::DrawMap(
  const gil::rgb8_view_t& image,
  const nav_msgs::OccupancyGrid& occupancyGrid,
  const Eigen::AlignedBox2i& boundingBox) const
{
  // Check that the image size is the same as the bounding box
  if (image.width() != boundingBox.sizes().x() ||
      image.height() != boundingBox.sizes().y()) {
    ROS_ERROR("Image size is inconsistent with the bounding box: "
              "image size: (%ld, %ld), bounding box size: (%d, %d)",
              image.width(), image.height(),
              boundingBox.sizes().x(), boundingBox.sizes().y());
    return false;
  }

  const int mapWidth = occupancyGrid.info.width;
  const int mapHeight = occupancyGrid.info.height;

  // Draw the grid cells on the image
  for (int row = 0; row < boundingBox.sizes().y(); ++row) {
    for (int col = 0; col < boundingBox.sizes().x(); ++col) {
      // Get the occupancy probability
      const int idx = (row + boundingBox.min().y()) * mapWidth
                      + (col + boundingBox.min().x());
      const std::int8_t probability = occupancyGrid.data[idx];

      // Skip the grid cell with unknown occupancy probability
      if (probability == -1)
        continue;

      // Convert the occupancy probability to the pixel intensity
      const auto intensity = static_cast<std::uint8_t>(
        (100 - probability) * 255.0 / 100.0);
      // Set the pixel intensity
      image(col, row) = gil::rgb8_pixel_t(intensity, intensity, intensity);
    }
  }

  return true;
}

// Draw the trajectory
bool OutputListenerNode::DrawTrajectory(
  const gil::rgb8_view_t& image,
  const std::vector<Eigen::Vector2f>& trajectory,
  const Eigen::Vector2f& origin,
  const float resolution) const
{
  if (trajectory.size() < 2)
    return true;

  const int lineWidth = this->mSettings.mTrajectoryLineWidth;
  const gil::rgb8_pixel_t lineColor {
    this->mSettings.mTrajectoryLineColor[0],
    this->mSettings.mTrajectoryLineColor[1],
    this->mSettings.mTrajectoryLineColor[2] };

  // Draw the trajectory lines to the image
  Eigen::Vector2i beginIdx {
    (trajectory.front().x() - origin.x()) / resolution,
    (trajectory.front().y() - origin.y()) / resolution };

  for (std::size_t i = 1; i < trajectory.size(); ++i) {
    Eigen::Vector2i endIdx {
      (trajectory[i].x() - origin.x()) / resolution,
      (trajectory[i].y() - origin.y()) / resolution };
    const auto indices = Bresenham(beginIdx, endIdx);

    for (const auto& interpolatedIdx : indices) {
      if (interpolatedIdx.x() < 0 ||
          interpolatedIdx.x() > image.width() - lineWidth ||
          interpolatedIdx.y() < 0 ||
          interpolatedIdx.y() > image.height() - lineWidth)
        continue;

      const auto& subView = gil::subimage_view(
        image, interpolatedIdx.x(), interpolatedIdx.y(), lineWidth, lineWidth);
      gil::fill_pixels(subView, lineColor);
    }

    beginIdx = endIdx;
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "output_listener_node");
  ros::NodeHandle nodeHandle;
  OutputListenerNode outputListener;

  if (!outputListener.Setup(nodeHandle)) {
    ROS_ERROR("OutputListenerNode: failed to setup the node");
    return EXIT_FAILURE;
  }

  ros::spin();

  ROS_INFO("OutputListenerNode: Exit");

  return EXIT_SUCCESS;
}
