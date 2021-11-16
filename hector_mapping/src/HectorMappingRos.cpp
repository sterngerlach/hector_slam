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

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include "map/GridMap.h"
#include "HectorMapMutex.h"
#include "HectorMappingRos.h"

#include "hw/bitstream_loader.hpp"
#include "hw/cma_manager.hpp"
#include "util/Parameter.hpp"
#include "util/Timer.hpp"

using namespace hectorslam;
using namespace hector_mapping;

HectorMappingRos::HectorMappingRos() :
  mDebugInfoProvider(nullptr),
  mHectorDrawings(nullptr),
  mLastGetMapUpdateIndex(-100),
  mInitialPoseSubscriber(nullptr),
  mInitialPoseFilter(nullptr),
  mTfBroadcaster(nullptr),
  mMapPublishThread(nullptr),
  mSlamProcessor(nullptr),
  mNumOfProcessedScans(0),
  mScanMatcherOption(hectorslam::ScanMatcherOption::Default),
  mDefaultScanMatcher(nullptr),
  mCorrelativeScanMatcher(nullptr),
  mFPGAScanMatcher(nullptr),
  mGaussNewtonScanMatcher(nullptr),
  mInitialPoseSet(false),
  mPauseScanProcessing(false)
{
  ros::NodeHandle privateNh { "~" };

  std::string mapTopic = "map";

  privateNh.param("pub_drawings",
                  this->mPubDrawings, false);
  privateNh.param("pub_debug_output",
                  this->mPubDebugOutput, false);
  privateNh.param("pub_map_odom_transform",
                  this->mPubMapToOdomTransform, true);
  privateNh.param("pub_odometry",
                  this->mPubOdometry, false);
  privateNh.param("advertise_map_service",
                  this->mAdvertiseMapService, true);
  privateNh.param("scan_subscriber_queue_size",
                  this->mScanSubscriberQueueSize, 5);

  privateNh.param("map_resolution", this->mMapResolution, 0.025);
  privateNh.param("map_size", this->mMapSize, 1024);
  privateNh.param("map_start_x", this->mMapStartX, 0.5);
  privateNh.param("map_start_y", this->mMapStartY, 0.5);
  privateNh.param("map_multi_res_levels", this->mMapMultiResLevels, 3);

  privateNh.param("update_factor_free",
                  this->mUpdateFactorFree, 0.4);
  privateNh.param("update_factor_occupied",
                  this->mUpdateFactorOccupied, 0.9);

  privateNh.param("map_update_distance_thresh",
                  this->mMapUpdateDistanceThreshold, 0.4);
  privateNh.param("map_update_angle_thresh",
                  this->mMapUpdateAngleThreshold, 0.9);

  privateNh.param("scan_topic",
                  this->mScanTopic, std::string("scan"));
  privateNh.param("sys_msg_topic",
                  this->mSysMsgTopic, std::string("syscommand"));
  privateNh.param("pose_update_topic",
                  this->mPoseUpdateTopic, std::string("poseupdate"));

  privateNh.param("use_tf_scan_transformation",
                  this->mUseTfScanTransformation, true);
  privateNh.param("use_tf_pose_start_estimate",
                  this->mUseTfPoseStartEstimate, false);
  privateNh.param("map_with_known_poses",
                  this->mMapWithKnownPoses, false);

  privateNh.param("base_frame",
                  this->mBaseFrame, std::string("base_link"));
  privateNh.param("map_frame",
                  this->mMapFrame, std::string("map"));
  privateNh.param("odom_frame",
                  this->mOdomFrame, std::string("odom"));

  privateNh.param("pub_map_scanmatch_transform",
                  this->mPubMapToScanMatchTransform, true);
  privateNh.param("tf_map_scanmatch_transform_frame_name",
                  this->mMapToScanMatchTransformFrameName,
                  std::string("scanmatcher_frame"));

  privateNh.param("output_timing", this->mTimingOutput, false);

  privateNh.param("map_pub_period", this->mMapPubPeriod, 2.0);

  double tmp = 0.0;
  privateNh.param("laser_min_dist", tmp, 0.4);
  this->mSquaredLaserMinDist = static_cast<float>(tmp * tmp);
  privateNh.param("laser_max_dist", tmp, 30.0);
  this->mSquaredLaserMaxDist = static_cast<float>(tmp * tmp);
  privateNh.param("laser_z_min_value", tmp, -1.0);
  this->mLaserZMinValue = static_cast<float>(tmp);
  privateNh.param("laser_z_max_value", tmp, 1.0);
  this->mLaserZMaxValue = static_cast<float>(tmp);

  if (this->mPubDrawings) {
    ROS_INFO("HectorMappingRos: publishing debug drawings");
    this->mHectorDrawings = std::make_unique<HectorDrawings>();
  }

  if (this->mPubDebugOutput) {
    ROS_INFO("HectorMappingRos: publishing debug info");
    this->mDebugInfoProvider = std::make_unique<HectorDebugInfoProvider>();
  }

  if (this->mPubOdometry)
    this->mOdometryPublisher = this->mNode.advertise<nav_msgs::Odometry>(
      "scanmatch_odom", 50);

  // Create a publisher to publish the metrics information
  this->mMetricsPublisher = this->mNode.advertise<
    hector_mapping::HectorMappingMetrics>("metrics", 10);

  // Initialize CMA manager and load bitstream for FPGA acceleration
  if (!this->setupFPGA(privateNh))
    std::exit(EXIT_FAILURE);

  // Get the scan matching option
  std::string scanMatcherOption;
  if (!GetParamFromNodeHandle(privateNh,
      "scan_matcher_option", scanMatcherOption))
    std::exit(EXIT_FAILURE);

  this->mScanMatcherOption = StringToScanMatcherOption(scanMatcherOption);
  if (this->mScanMatcherOption == ScanMatcherOption::Unknown) {
    ROS_ERROR("Unknown scan matcher option: %s", scanMatcherOption.c_str());
    std::exit(EXIT_FAILURE);
  }

  // Initialize the scan matcher
  if (!this->initializeScanMatcher(privateNh)) {
    ROS_ERROR("Failed to initialize the scan matcher");
    std::exit(EXIT_FAILURE);
  }

  // Create the callback for scan matching
  const auto callback = [this](
    const Eigen::Vector3f& initialWorldPose,
    GridMapUtil& gridMapUtil,
    const hectorslam::DataContainer& dataContainer,
    Eigen::Matrix3f& covMatrix, const int mapIndex) {
    return this->scanMatchCallback(
      initialWorldPose, gridMapUtil, dataContainer, covMatrix, mapIndex); };

  this->mSlamProcessor = std::make_unique<HectorSlamProcessor>(
    static_cast<float>(this->mMapResolution),
    this->mMapSize, this->mMapSize,
    Eigen::Vector2f(this->mMapStartX, this->mMapStartY),
    this->mMapMultiResLevels,
    callback, this->mHectorDrawings.get(), this->mDebugInfoProvider.get());

  this->mSlamProcessor->setUpdateFactorFree(
    this->mUpdateFactorFree);
  this->mSlamProcessor->setUpdateFactorOccupied(
    this->mUpdateFactorOccupied);
  this->mSlamProcessor->setMapUpdateMinDistDiff(
    this->mMapUpdateDistanceThreshold);
  this->mSlamProcessor->setMapUpdateMinAngleDiff(
    this->mMapUpdateAngleThreshold);

  // int mapLevels = this->mSlamProcessor->getMapLevels();
  int mapLevels = 1;

  for (int i = 0; i < mapLevels; ++i) {
    this->mMapPubContainer.push_back(MapPublisherContainer());
    this->mSlamProcessor->addMapMutex(i, new HectorMapMutex());

    std::string mapTopicStr = mapTopic;

    if (i != 0)
      mapTopicStr.append("_" + boost::lexical_cast<std::string>(i));

    std::string mapMetaTopicStr = mapTopicStr;
    mapMetaTopicStr.append("_metadata");

    MapPublisherContainer& tmp = this->mMapPubContainer[i];
    tmp.mapPublisher_ = this->mNode.advertise<nav_msgs::OccupancyGrid>(
      mapTopicStr, 1, true);
    tmp.mapMetadataPublisher_ = this->mNode.advertise<nav_msgs::MapMetaData>(
      mapMetaTopicStr, 1, true);

    if (i == 0 && this->mAdvertiseMapService)
      tmp.dynamicMapServiceServer_ = this->mNode.advertiseService(
        "dynamic_map", &HectorMappingRos::mapCallback, this);

    this->setServiceGetMapData(tmp.map_, this->mSlamProcessor->getGridMap(i));

    if (i == 0)
      this->mMapPubContainer[i].mapMetadataPublisher_.publish(
        this->mMapPubContainer[i].map_.map.info);
  }

  // Initialize services
  this->mResetMapService = this->mNode.advertiseService(
    "reset_map", &HectorMappingRos::resetMapCallback, this);
  this->mRestartHectorService = this->mNode.advertiseService(
    "restart_mapping_with_new_pose",
    &HectorMappingRos::restartHectorCallback, this);
  this->mToggleScanProcessingService = this->mNode.advertiseService(
    "pause_mapping", &HectorMappingRos::pauseMapCallback, this);

  ROS_INFO("HectorMappingRos::mBaseFrame: %s", this->mBaseFrame.c_str());
  ROS_INFO("HectorMappingRos::mMapFrame: %s", this->mMapFrame.c_str());
  ROS_INFO("HectorMappingRos::mOdomFrame: %s", this->mOdomFrame.c_str());
  ROS_INFO("HectorMappingRos::mScanTopic: %s", this->mScanTopic.c_str());
  ROS_INFO("HectorMappingRos::mUseTfScanTransformation: %s",
           this->mUseTfScanTransformation ? "true" : "false");
  ROS_INFO("HectorMappingRos::mPubMapToOdomTransform: %s",
           this->mPubMapToOdomTransform ? "true" : "false");
  ROS_INFO("HectorMappingRos::mScanSubscriberQueueSize: %d",
           this->mScanSubscriberQueueSize);
  ROS_INFO("HectorMappingRos::mMapPubPeriod: %f",
           this->mMapPubPeriod);
  ROS_INFO("HectorMappingRos::mUpdateFactorFree: %f",
           this->mUpdateFactorFree);
  ROS_INFO("HectorMappingRos::mUpdateFactorOccupied: %f",
           this->mUpdateFactorOccupied);
  ROS_INFO("HectorMappingRos::mMapUpdateDistanceThreshold: %f ",
           this->mMapUpdateDistanceThreshold);
  ROS_INFO("HectorMappingRos::mMapUpdateAngleThreshold: %f",
           this->mMapUpdateAngleThreshold);
  ROS_INFO("HectorMappingRos::mLaserZMinValue: %f", this->mLaserZMinValue);
  ROS_INFO("HectorMappingRos::mLaserZMaxValue: %f", this->mLaserZMaxValue);

  this->mScanSubscriber = this->mNode.subscribe(
    this->mScanTopic, this->mScanSubscriberQueueSize,
    &HectorMappingRos::scanCallback, this);
  this->mSysMsgSubscriber = this->mNode.subscribe(
    this->mSysMsgTopic, 2,
    &HectorMappingRos::sysMsgCallback, this);

  this->mPoseUpdatePublisher = this->mNode.advertise<
    geometry_msgs::PoseWithCovarianceStamped>(
      this->mPoseUpdateTopic, 1, false);
  this->mPosePublisher = this->mNode.advertise<
    geometry_msgs::PoseStamped>("slam_out_pose", 1, false);
  this->mScanPointCloudPublisher = this->mNode.advertise<
    sensor_msgs::PointCloud>("slam_cloud", 1, false);

  this->mTfBroadcaster = std::make_unique<tf::TransformBroadcaster>();
  ROS_ASSERT(this->mTfBroadcaster != nullptr);

  this->mInitialPoseSubscriber = std::make_unique<message_filters::Subscriber<
    geometry_msgs::PoseWithCovarianceStamped>>(this->mNode, "initialpose", 2);
  this->mInitialPoseFilter = std::make_unique<tf::MessageFilter<
    geometry_msgs::PoseWithCovarianceStamped>>(
      *this->mInitialPoseSubscriber, this->mTf, this->mMapFrame, 2);
  this->mInitialPoseFilter->registerCallback(boost::bind(
    &HectorMappingRos::initialPoseCallback, this, _1));

  this->mMapPublishThread = std::make_unique<boost::thread>(
    boost::bind(&HectorMappingRos::publishMapLoop, this,
      this->mMapPubPeriod));
}

bool HectorMappingRos::setupFPGA(ros::NodeHandle& nh)
{
  // Get the option to enable FPGA acceleration
  bool enableAcceleration;
  if (!GetParamFromNodeHandle(nh,
      "enable_fpga_acceleration", enableAcceleration))
    return false;

  if (!enableAcceleration)
    return true;

  // Initialize the CMA manager
  auto* pCMAManager = hw::CMAMemoryManager::Instance();
  if (!pCMAManager->Load()) {
    ROS_ERROR("Failed to initialize CMA manager");
    return false;
  }

  // Load the bitstream path
  std::string bitstreamFileName;
  if (!GetParamFromNodeHandle(nh, "bitstream_file_name", bitstreamFileName))
    return false;

  // Load the bitstream
  hw::BitstreamLoader bitstreamLoader;
  if (!bitstreamLoader.Load(bitstreamFileName)) {
    ROS_ERROR("Failed to load the bitstream: %s", bitstreamFileName.c_str());
    return false;
  }

  return true;
}

bool HectorMappingRos::initializeScanMatcher(ros::NodeHandle& nh)
{
  switch (this->mScanMatcherOption) {
    case ScanMatcherOption::Default:
      this->mDefaultScanMatcher = DefaultScanMatcher::Create(
        nh, this->mHectorDrawings.get(), this->mDebugInfoProvider.get());
      return this->mDefaultScanMatcher != nullptr;

    case ScanMatcherOption::GaussNewton:
      this->mGaussNewtonScanMatcher = ScanMatcherGaussNewton::Create(
        nh, this->mHectorDrawings.get(), this->mDebugInfoProvider.get());
      return this->mGaussNewtonScanMatcher != nullptr;

    case ScanMatcherOption::Correlative:
      this->mCorrelativeScanMatcher = ScanMatcherCorrelative::Create(
        nh, this->mHectorDrawings.get(), this->mDebugInfoProvider.get());
      return this->mCorrelativeScanMatcher != nullptr;

    case ScanMatcherOption::CorrelativeFPGA:
      this->mFPGAScanMatcher = ScanMatcherFPGA::Create(
        nh, this->mHectorDrawings.get(), this->mDebugInfoProvider.get());
      return this->mFPGAScanMatcher != nullptr;

    case ScanMatcherOption::GaussNewtonAfterCorrelative:
      this->mCorrelativeScanMatcher = ScanMatcherCorrelative::Create(
        nh, this->mHectorDrawings.get(), this->mDebugInfoProvider.get());
      this->mGaussNewtonScanMatcher = ScanMatcherGaussNewton::Create(
        nh, this->mHectorDrawings.get(), this->mDebugInfoProvider.get());
      return this->mCorrelativeScanMatcher != nullptr &&
             this->mGaussNewtonScanMatcher != nullptr;

    case ScanMatcherOption::GaussNewtonAfterCorrelativeFPGA:
      this->mFPGAScanMatcher = ScanMatcherFPGA::Create(
        nh, this->mHectorDrawings.get(), this->mDebugInfoProvider.get());
      this->mGaussNewtonScanMatcher = ScanMatcherGaussNewton::Create(
        nh, this->mHectorDrawings.get(), this->mDebugInfoProvider.get());
      return this->mFPGAScanMatcher != nullptr &&
             this->mGaussNewtonScanMatcher != nullptr;
  }

  return false;
}

Eigen::Vector3f HectorMappingRos::scanMatchCallback(
  const Eigen::Vector3f& initialWorldPose,
  GridMapUtil& gridMapUtil,
  const hectorslam::DataContainer& dataContainer,
  Eigen::Matrix3f& covMatrix, const int mapIndex)
{
  switch (this->mScanMatcherOption) {
    case ScanMatcherOption::Default: {
      ROS_ASSERT(this->mDefaultScanMatcher != nullptr);
      const int maxIterations = mapIndex == 0 ? 5 : 3;
      return this->mDefaultScanMatcher->matchData(
        initialWorldPose, gridMapUtil, dataContainer,
        covMatrix, maxIterations);
    }

    case ScanMatcherOption::GaussNewton: {
      ROS_ASSERT(this->mGaussNewtonScanMatcher != nullptr);
      const bool fineMatching = mapIndex == 0;
      ScanMatcherGaussNewtonMetrics metrics;

      metrics.stamp = ros::WallTime::now().toNSec();
      metrics.map_resolution_level = mapIndex;
      const Eigen::Vector3f estimatedPose =
        this->mGaussNewtonScanMatcher->MatchScans(
          initialWorldPose, gridMapUtil, dataContainer,
          fineMatching, &covMatrix, &metrics);
      this->mMetricsMsg.scan_matcher_gauss_newton_metrics.push_back(metrics);

      return estimatedPose;
    }

    case ScanMatcherOption::Correlative: {
      ROS_ASSERT(this->mCorrelativeScanMatcher != nullptr);
      const float scoreMin = 0.0f;
      const float correspondenceRatioMin = 0.0f;
      ScanMatcherCorrelativeMetrics metrics;

      metrics.stamp = ros::WallTime::now().toNSec();
      metrics.map_resolution_level = mapIndex;
      const Eigen::Vector3f estimatedPose =
        this->mCorrelativeScanMatcher->MatchScans(
          initialWorldPose, gridMapUtil, dataContainer,
          scoreMin, correspondenceRatioMin, &covMatrix, &metrics);
      this->mMetricsMsg.scan_matcher_correlative_metrics.push_back(metrics);

      return estimatedPose;
    }

    case ScanMatcherOption::CorrelativeFPGA: {
      ROS_ASSERT(this->mFPGAScanMatcher != nullptr);
      const float scoreMin = 0.0f;
      const float correspondenceRatioMin = 0.0f;
      ScanMatcherFPGAMetrics metrics;

      metrics.stamp = ros::WallTime::now().toNSec();
      metrics.map_resolution_level = mapIndex;
      const Eigen::Vector3f estimatedPose = this->mFPGAScanMatcher->MatchScans(
        initialWorldPose, gridMapUtil, dataContainer,
        scoreMin, correspondenceRatioMin, &covMatrix, &metrics);
      this->mMetricsMsg.scan_matcher_fpga_metrics.push_back(metrics);

      return estimatedPose;
    }

    case ScanMatcherOption::GaussNewtonAfterCorrelative: {
      ROS_ASSERT(this->mCorrelativeScanMatcher != nullptr);
      ROS_ASSERT(this->mGaussNewtonScanMatcher != nullptr);
      const bool fineMatching = mapIndex == 0;
      const float scoreMin = 0.0f;
      const float correspondenceRatioMin = 0.0f;
      ScanMatcherCorrelativeMetrics correlativeMetrics;
      ScanMatcherGaussNewtonMetrics gaussNewtonMetrics;

      correlativeMetrics.stamp = ros::WallTime::now().toNSec();
      correlativeMetrics.map_resolution_level = mapIndex;
      Eigen::Vector3f estimatedPose = this->mCorrelativeScanMatcher->MatchScans(
        initialWorldPose, gridMapUtil, dataContainer,
        scoreMin, correspondenceRatioMin, nullptr, &correlativeMetrics);

      gaussNewtonMetrics.stamp = ros::WallTime::now().toNSec();
      gaussNewtonMetrics.map_resolution_level = mapIndex;
      estimatedPose = this->mGaussNewtonScanMatcher->MatchScans(
        estimatedPose, gridMapUtil, dataContainer,
        fineMatching, &covMatrix, &gaussNewtonMetrics);

      this->mMetricsMsg.scan_matcher_correlative_metrics.push_back(
        correlativeMetrics);
      this->mMetricsMsg.scan_matcher_gauss_newton_metrics.push_back(
        gaussNewtonMetrics);

      return estimatedPose;
    }

    case ScanMatcherOption::GaussNewtonAfterCorrelativeFPGA: {
      ROS_ASSERT(this->mFPGAScanMatcher != nullptr);
      ROS_ASSERT(this->mGaussNewtonScanMatcher != nullptr);
      const bool fineMatching = mapIndex == 0;
      const float scoreMin = 0.0f;
      const float correspondenceRatioMin = 0.0f;
      ScanMatcherFPGAMetrics fpgaMetrics;
      ScanMatcherGaussNewtonMetrics gaussNewtonMetrics;

      fpgaMetrics.stamp = ros::WallTime::now().toNSec();
      fpgaMetrics.map_resolution_level = mapIndex;
      Eigen::Vector3f estimatedPose = this->mFPGAScanMatcher->MatchScans(
        initialWorldPose, gridMapUtil, dataContainer,
        scoreMin, correspondenceRatioMin, nullptr, &fpgaMetrics);

      gaussNewtonMetrics.stamp = ros::WallTime::now().toNSec();
      gaussNewtonMetrics.map_resolution_level = mapIndex;
      estimatedPose = this->mGaussNewtonScanMatcher->MatchScans(
        estimatedPose, gridMapUtil, dataContainer,
        fineMatching, &covMatrix, &gaussNewtonMetrics);

      this->mMetricsMsg.scan_matcher_fpga_metrics.push_back(fpgaMetrics);
      this->mMetricsMsg.scan_matcher_gauss_newton_metrics.push_back(
        gaussNewtonMetrics);

      return estimatedPose;
    }
  }

  return initialWorldPose;
}

void HectorMappingRos::scanCallback(const sensor_msgs::LaserScan& scan)
{
  if (this->mPauseScanProcessing)
    return;

  // Initialize the metrics information
  this->mMetricsMsg = hector_mapping::HectorMappingMetrics();
  // Measure the time for processing the current frame (scan)
  Timer outerTimer;

  if (++this->mNumOfProcessedScans % 10 == 0)
    ROS_INFO("Processing the frame: %d", this->mNumOfProcessedScans);

  if (this->mHectorDrawings.get() != nullptr)
    this->mHectorDrawings->setTime(scan.header.stamp);

  ros::WallTime startTime = ros::WallTime::now();

  if (!this->mUseTfScanTransformation) {
    // Measure the time for preprocessing the current scan
    Timer timer;
    // If we are not using the tf tree to find the transform between the
    // base frame and laser frame, then just convert the laser scan to our data
    // container and process the update based on our last pose estimate
    hectorslam::DataContainer scanContainer;
    this->rosLaserScanToDataContainer(scan, scanContainer,
      this->mSlamProcessor->getScaleToMap());
    this->mMetricsMsg.scan_preprocessing_time.fromNSec(
      timer.ElapsedNanoseconds());

    this->mSlamProcessor->update(scanContainer,
      this->mSlamProcessor->getLastScanMatchPose(), &this->mMetricsMsg);
  } else {
    // If we are using the tf tree to find the transform between the
    // base frame and laser frame, let's get that transform
    tf::StampedTransform laserTransform;
    if (this->mTf.waitForTransform(this->mBaseFrame, scan.header.frame_id,
        scan.header.stamp, ros::Duration(0.5))) {
      this->mTf.lookupTransform(this->mBaseFrame, scan.header.frame_id,
        scan.header.stamp, laserTransform);
    } else {
      ROS_INFO("lookupTransform %s to %s timed out. "
               "Could not transform laser scan into base_frame.",
               this->mBaseFrame.c_str(), scan.header.frame_id.c_str());
      return;
    }

    // Measure the time for preprocessing the current scan
    Timer timer;

    // Convert the laser scan to point cloud
    laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud laserPointCloud;
    projector.projectLaser(scan, laserPointCloud, 30.0);

    // Publish the point cloud if there are any subscribers
    if (this->mScanPointCloudPublisher.getNumSubscribers() > 0)
      this->mScanPointCloudPublisher.publish(laserPointCloud);

    // Convert the point cloud to our data container
    hectorslam::DataContainer scanContainer;
    this->rosPointCloudToDataContainer(
      laserPointCloud, laserTransform, scanContainer,
      this->mSlamProcessor->getScaleToMap());
    this->mMetricsMsg.scan_preprocessing_time.fromNSec(
      timer.ElapsedNanoseconds());

    // Now let's choose the initial pose estimate for our slam process update
    Eigen::Vector3f startEstimate { Eigen::Vector3f::Zero() };
    if (this->mInitialPoseSet) {
      // User has requested a pose reset
      this->mInitialPoseSet = false;
      startEstimate = this->mInitialPose;
    } else if (this->mUseTfPoseStartEstimate) {
      // Initial pose estimate comes from the tf tree
      try {
        tf::StampedTransform stamped_pose;

        this->mTf.waitForTransform(this->mMapFrame, this->mBaseFrame,
          scan.header.stamp, ros::Duration(0.5));
        this->mTf.lookupTransform(this->mMapFrame, this->mBaseFrame,
          scan.header.stamp, stamped_pose);

        startEstimate[0] = stamped_pose.getOrigin().getX();
        startEstimate[1] = stamped_pose.getOrigin().getY();
        startEstimate[2] = tf::getYaw(stamped_pose.getRotation());
      } catch(tf::TransformException e) {
        ROS_ERROR("Transform from %s to %s failed\n",
                  this->mMapFrame.c_str(), this->mBaseFrame.c_str());
        startEstimate = this->mSlamProcessor->getLastScanMatchPose();
      }
    } else {
      // If none of the above, initial pose is simply the last estimated pose
      startEstimate = this->mSlamProcessor->getLastScanMatchPose();
    }

    // If "mMapWithKnownPoses" is enabled, we assume that startEstimate
    // is precise and doesn't need to be refined
    if (this->mMapWithKnownPoses)
      this->mSlamProcessor->update(scanContainer, startEstimate,
        &this->mMetricsMsg, true);
    else
      this->mSlamProcessor->update(scanContainer, startEstimate,
        &this->mMetricsMsg);
  }

  // If the debug flag "mTimingOutput" is enabled, print how long this last
  // iteration took
  if (this->mTimingOutput) {
    ros::WallDuration duration = ros::WallTime::now() - startTime;
    ROS_INFO("HectorSLAM Iter took: %f milliseconds",
             duration.toSec() * 1000.0f);
  }

  // If we're just building a map with known poses, we're finished now.
  // Code below this point publishes the localization results.
  if (this->mMapWithKnownPoses)
    return;

  Timer timer;

  this->mPoseInfoContainer.update(
    this->mSlamProcessor->getLastScanMatchPose(),
    this->mSlamProcessor->getLastScanMatchCovariance(),
    scan.header.stamp, this->mMapFrame);

  // Publish pose with and without covariances
  this->mPoseUpdatePublisher.publish(
    this->mPoseInfoContainer.getPoseWithCovarianceStamped());
  this->mPosePublisher.publish(this->mPoseInfoContainer.getPoseStamped());

  // Publish odometry if enabled
  if (this->mPubOdometry) {
    const auto& poseAndCovariance =
      this->mPoseInfoContainer.getPoseWithCovarianceStamped();
    nav_msgs::Odometry odometry;
    odometry.pose = poseAndCovariance.pose;
    odometry.header = poseAndCovariance.header;
    odometry.child_frame_id = this->mBaseFrame;
    this->mOdometryPublisher.publish(odometry);
  }

  // Publish the map->odom transform if enabled
  if (this->mPubMapToOdomTransform) {
    tf::StampedTransform odomToBase;
    try {
      this->mTf.waitForTransform(this->mOdomFrame, this->mBaseFrame,
        scan.header.stamp, ros::Duration(0.5));
      this->mTf.lookupTransform(this->mOdomFrame, this->mBaseFrame,
        scan.header.stamp, odomToBase);
    } catch (tf::TransformException e) {
      ROS_ERROR("Transform failed during publishing of map_odom transform: %s",
                e.what());
      odomToBase.setIdentity();
    }

    const tf::Transform mapToOdom = this->mPoseInfoContainer.getTfTransform()
      * odomToBase.inverse();
    this->mTfBroadcaster->sendTransform(tf::StampedTransform(
      mapToOdom, scan.header.stamp,
      this->mMapFrame, this->mOdomFrame));
  }

  // Publish the transform from map to estimated pose (if enabled)
  if (this->mPubMapToScanMatchTransform)
    this->mTfBroadcaster->sendTransform(tf::StampedTransform(
      this->mPoseInfoContainer.getTfTransform(), scan.header.stamp,
      this->mMapFrame, this->mMapToScanMatchTransformFrameName));

  // Fill and publish the metrics information
  this->mMetricsMsg.result_publish_time.fromNSec(timer.ElapsedNanoseconds());
  this->mMetricsMsg.processing_time.fromNSec(outerTimer.ElapsedNanoseconds());

  this->mMetricsMsg.scan_matcher_option =
    static_cast<std::int32_t>(this->mScanMatcherOption);
  this->mMetricsMsg.stamp = ros::WallTime::now().toNSec();
  this->mMetricsMsg.scan_time_stamp = scan.header.stamp.toNSec();

  // Publish the metrics information
  if (this->mMetricsPublisher.getNumSubscribers() > 0)
    this->mMetricsPublisher.publish(this->mMetricsMsg);
}

void HectorMappingRos::sysMsgCallback(const std_msgs::String& sysMsg)
{
  ROS_INFO("Hector SLAM node received a message: %s", sysMsg.data.c_str());

  if (sysMsg.data == "reset") {
    ROS_INFO("Hector SLAM node is reset");
    this->mSlamProcessor->reset();
  } else if (sysMsg.data == "shutdown") {
    ROS_INFO("Hector SLAM node is shutting down");
    ros::requestShutdown();
  }
}

bool HectorMappingRos::mapCallback(
  nav_msgs::GetMap::Request& req,
  nav_msgs::GetMap::Response& res)
{
  ROS_INFO("HectorMappingRos: Map service called");
  res = this->mMapPubContainer[0].map_;
  return true;
}

bool HectorMappingRos::resetMapCallback(
  std_srvs::Trigger::Request& req,
  std_srvs::Trigger::Response& res)
{
  ROS_INFO("HectorMappingRos: Reset map service called");
  this->mSlamProcessor->reset();
  return true;
}

bool HectorMappingRos::restartHectorCallback(
  hector_mapping::ResetMapping::Request& req,
  hector_mapping::ResetMapping::Response& res)
{
  // Reset map
  ROS_INFO("HectorMappingRos: Reset map");
  this->mSlamProcessor->reset();

  // Reset pose
  this->resetPose(req.initial_pose);

  // Unpause node (in case it is paused)
  this->toggleMappingPause(false);

  // Return success
  return true;
}

bool HectorMappingRos::pauseMapCallback(
  std_srvs::SetBool::Request& req,
  std_srvs::SetBool::Response& res)
{
  this->toggleMappingPause(req.data);
  res.success = true;
  return true;
}

void HectorMappingRos::publishMap(MapPublisherContainer& mapPublisher,
                                  const hectorslam::GridMap& gridMap,
                                  ros::Time timestamp,
                                  MapLockerInterface* mapMutex)
{
  nav_msgs::GetMap::Response& map = mapPublisher.map_;

  // Only update map if it changed
  if (this->mLastGetMapUpdateIndex != gridMap.getUpdateIndex()) {
    const int size = gridMap.getSizeX() * gridMap.getSizeY();

    std::vector<int8_t>& data = map.map.data;

    // std::vector contents are guaranteed to be contiguous,
    // use memset to set all to unknown to save time in loop
    std::memset(&data[0], -1, sizeof(std::int8_t) * size);

    if (mapMutex != nullptr)
      mapMutex->lockMap();

    for (int i = 0; i < size; ++i)
      if (gridMap.isFree(i))
        data[i] = 0;
      else if (gridMap.isOccupied(i))
        data[i] = 100;

    this->mLastGetMapUpdateIndex = gridMap.getUpdateIndex();

    if (mapMutex != nullptr)
      mapMutex->unlockMap();
  }

  map.map.header.stamp = timestamp;

  mapPublisher.mapPublisher_.publish(map.map);
}

void HectorMappingRos::rosLaserScanToDataContainer(
  const sensor_msgs::LaserScan& scan,
  hectorslam::DataContainer& dataContainer,
  float scaleToMap)
{
  const std::size_t size = scan.ranges.size();

  float angle = scan.angle_min;

  dataContainer.clear();
  dataContainer.setOrigo(Eigen::Vector2f::Zero());

  const float maxRangeForContainer = scan.range_max - 0.1f;

  for (std::size_t i = 0; i < size; ++i) {
    float dist = scan.ranges[i];

    if (dist > scan.range_min && dist < maxRangeForContainer) {
      dist *= scaleToMap;
      dataContainer.add(Eigen::Vector2f(
        std::cos(angle) * dist, std::sin(angle) * dist));
    }

    angle += scan.angle_increment;
  }
}

void HectorMappingRos::rosPointCloudToDataContainer(
  const sensor_msgs::PointCloud& pointCloud,
  const tf::StampedTransform& laserTransform,
  hectorslam::DataContainer& dataContainer,
  float scaleToMap)
{
  const std::size_t size = pointCloud.points.size();

  dataContainer.clear();

  const tf::Vector3 laserPos = laserTransform.getOrigin();
  dataContainer.setOrigo(Eigen::Vector2f(
    laserPos.x(), laserPos.y()) * scaleToMap);

  for (std::size_t i = 0; i < size; ++i) {
    const geometry_msgs::Point32& currPoint = pointCloud.points[i];

    const float distSquared = currPoint.x * currPoint.x
                              + currPoint.y * currPoint.y;

    if (distSquared > this->mSquaredLaserMinDist &&
        distSquared < this->mSquaredLaserMaxDist) {
      if (currPoint.x < 0.0f && distSquared < 0.50f)
        continue;

      const tf::Vector3 pointPosBaseFrame = laserTransform
        * tf::Vector3(currPoint.x, currPoint.y, currPoint.z);
      const float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos.z();

      if (pointPosLaserFrameZ > this->mLaserZMinValue &&
          pointPosLaserFrameZ < this->mLaserZMaxValue)
        dataContainer.add(Eigen::Vector2f(
          pointPosBaseFrame.x(), pointPosBaseFrame.y()) * scaleToMap);
    }
  }
}

void HectorMappingRos::setServiceGetMapData(
  nav_msgs::GetMap::Response& map_,
  const hectorslam::GridMap& gridMap)
{
  Eigen::Vector2f mapOrigin = gridMap.getWorldCoords(Eigen::Vector2f::Zero());
  mapOrigin.array() -= gridMap.getCellLength() * 0.5f;

  map_.map.info.origin.position.x = mapOrigin.x();
  map_.map.info.origin.position.y = mapOrigin.y();
  map_.map.info.origin.orientation.w = 1.0;

  map_.map.info.resolution = gridMap.getCellLength();

  map_.map.info.width = gridMap.getSizeX();
  map_.map.info.height = gridMap.getSizeY();

  map_.map.header.frame_id = this->mMapFrame;
  map_.map.data.resize(map_.map.info.width * map_.map.info.height);
}

/*
void HectorMappingRos::setStaticMapData(const nav_msgs::OccupancyGrid& map)
{
  float cell_length = map.info.resolution;
  Eigen::Vector2f mapOrigin { map.info.origin.position.x + cell_length * 0.5f,
                              map.info.origin.position.y + cell_length * 0.5f };

  int map_size_x = map.info.width;
  int map_size_y = map.info.height;

  this->slamProcessor = new hectorslam::HectorSlamProcessor(
    cell_length, map_size_x, map_size_y,
    Eigen::Vector2f(0.0f, 0.0f), 1, hectorDrawings, debugInfoProvider);
}
*/

void HectorMappingRos::publishMapLoop(double map_pub_period)
{
  ros::Rate r { 1.0 / map_pub_period };

  while (ros::ok()) {
    // ros::WallTime t1 = ros::WallTime::now();
    ros::Time mapTime = ros::Time::now();
    // this->publishMap(this->mapPubContainer[2],
    //   this->slamProcessor->getGridMap(2), mapTime);
    // this->publishMap(this->mapPubContainer[1],
    //   this->slamProcessor->getGridMap(1), mapTime);
    this->publishMap(this->mMapPubContainer[0],
                     this->mSlamProcessor->getGridMap(0),
                     mapTime,
                     this->mSlamProcessor->getMapMutex(0));

    // ros::WallDuration t2 = ros::WallTime::now() - t1;

    // std::cout << "time s: " << t2.toSec();
    // ROS_INFO("HectorMappingRos: ms: %4.2f", t2.toSec() * 1000.0f);

    r.sleep();
  }
}

void HectorMappingRos::initialPoseCallback(
  const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  this->resetPose(msg->pose.pose);
}

void HectorMappingRos::toggleMappingPause(bool pause)
{
  // Pause/unpause
  if (pause && !this->mPauseScanProcessing)
    ROS_INFO("HectorMappingRos: Mapping paused");
  else if (!pause && this->mPauseScanProcessing)
    ROS_INFO("HectorMappingRos: Mapping no longer paused");
  this->mPauseScanProcessing = pause;
}

void HectorMappingRos::resetPose(const geometry_msgs::Pose &pose)
{
  this->mInitialPoseSet = true;
  this->mInitialPose = Eigen::Vector3f(
    pose.position.x, pose.position.y,
    util::getYawFromQuat(pose.orientation));

  ROS_INFO("HectorMappingRos: Setting initial pose with world coords "
           "x: %f y: %f yaw: %f",
           this->mInitialPose[0], this->mInitialPose[1], this->mInitialPose[2]);
}
