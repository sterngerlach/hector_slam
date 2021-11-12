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

#include "HectorMappingRos.h"

#include "map/GridMap.h"

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include "sensor_msgs/PointCloud2.h"

#include "HectorDrawings.h"
#include "HectorDebugInfoProvider.h"
#include "HectorMapMutex.h"

#include "hw/bitstream_loader.hpp"
#include "hw/cma_manager.hpp"
#include "util/Parameter.hpp"
#include "util/Timer.hpp"

#ifndef TF_SCALAR_H
  typedef btScalar tfScalar;
#endif

using namespace hectorslam;
using namespace hector_mapping;

HectorMappingRos::HectorMappingRos() :
  debugInfoProvider(nullptr),
  hectorDrawings(nullptr),
  lastGetMapUpdateIndex(-100),
  tfB_(nullptr),
  map__publish_thread_(nullptr),
  slamProcessor(nullptr),
  mNumOfProcessedScans(0),
  mScanMatcherOption(hectorslam::ScanMatcherOption::Default),
  mDefaultScanMatcher(nullptr),
  mCorrelativeScanMatcher(nullptr),
  mFPGAScanMatcher(nullptr),
  mGaussNewtonScanMatcher(nullptr),
  initial_pose_set_(false),
  pause_scan_processing_(false)
{
  ros::NodeHandle private_nh_("~");

  std::string mapTopic_ = "map";

  private_nh_.param("pub_drawings", p_pub_drawings, false);
  private_nh_.param("pub_debug_output", p_pub_debug_output_, false);
  private_nh_.param("pub_map_odom_transform", p_pub_map_odom_transform_,true);
  private_nh_.param("pub_odometry", p_pub_odometry_,false);
  private_nh_.param("advertise_map_service", p_advertise_map_service_,true);
  private_nh_.param("scan_subscriber_queue_size", p_scan_subscriber_queue_size_, 5);

  private_nh_.param("map_resolution", p_map_resolution_, 0.025);
  private_nh_.param("map_size", p_map_size_, 1024);
  private_nh_.param("map_start_x", p_map_start_x_, 0.5);
  private_nh_.param("map_start_y", p_map_start_y_, 0.5);
  private_nh_.param("map_multi_res_levels", p_map_multi_res_levels_, 3);

  private_nh_.param("update_factor_free", p_update_factor_free_, 0.4);
  private_nh_.param("update_factor_occupied", p_update_factor_occupied_, 0.9);

  private_nh_.param("map_update_distance_thresh", p_map_update_distance_threshold_, 0.4);
  private_nh_.param("map_update_angle_thresh", p_map_update_angle_threshold_, 0.9);

  private_nh_.param("scan_topic", p_scan_topic_, std::string("scan"));
  private_nh_.param("sys_msg_topic", p_sys_msg_topic_, std::string("syscommand"));
  private_nh_.param("pose_update_topic", p_pose_update_topic_, std::string("poseupdate"));

  private_nh_.param("use_tf_scan_transformation", p_use_tf_scan_transformation_,true);
  private_nh_.param("use_tf_pose_start_estimate", p_use_tf_pose_start_estimate_,false);
  private_nh_.param("map_with_known_poses", p_map_with_known_poses_, false);

  private_nh_.param("base_frame", p_base_frame_, std::string("base_link"));
  private_nh_.param("map_frame", p_map_frame_, std::string("map"));
  private_nh_.param("odom_frame", p_odom_frame_, std::string("odom"));

  private_nh_.param("pub_map_scanmatch_transform", p_pub_map_scanmatch_transform_,true);
  private_nh_.param("tf_map_scanmatch_transform_frame_name", p_tf_map_scanmatch_transform_frame_name_, std::string("scanmatcher_frame"));

  private_nh_.param("output_timing", p_timing_output_,false);

  private_nh_.param("map_pub_period", p_map_pub_period_, 2.0);

  double tmp = 0.0;
  private_nh_.param("laser_min_dist", tmp, 0.4);
  p_sqr_laser_min_dist_ = static_cast<float>(tmp*tmp);

  private_nh_.param("laser_max_dist", tmp, 30.0);
  p_sqr_laser_max_dist_ = static_cast<float>(tmp*tmp);

  private_nh_.param("laser_z_min_value", tmp, -1.0);
  p_laser_z_min_value_ = static_cast<float>(tmp);

  private_nh_.param("laser_z_max_value", tmp, 1.0);
  p_laser_z_max_value_ = static_cast<float>(tmp);

  if (p_pub_drawings)
  {
    ROS_INFO("HectorSM publishing debug drawings");
    hectorDrawings = new HectorDrawings();
  }

  if(p_pub_debug_output_)
  {
    ROS_INFO("HectorSM publishing debug info");
    debugInfoProvider = new HectorDebugInfoProvider();
  }

  if(p_pub_odometry_)
  {
    odometryPublisher_ = node_.advertise<nav_msgs::Odometry>("scanmatch_odom", 50);
  }

  // Create a publisher to publish the metrics information
  this->mMetricsPublisher = this->node_.advertise<
    hector_mapping::HectorMappingMetrics>("metrics", 10);

  // Initialize CMA manager and load bitstream for FPGA acceleration
  if (!this->setupFPGA(private_nh_))
    std::exit(EXIT_FAILURE);

  // Get the scan matching option
  std::string scanMatcherOption;
  if (!GetParamFromNodeHandle(private_nh_,
      "scan_matcher_option", scanMatcherOption))
    std::exit(EXIT_FAILURE);

  this->mScanMatcherOption = StringToScanMatcherOption(scanMatcherOption);
  if (this->mScanMatcherOption == ScanMatcherOption::Unknown) {
    ROS_ERROR("Unknown scan matcher option: %s", scanMatcherOption.c_str());
    std::exit(EXIT_FAILURE);
  }

  // Initialize the scan matcher
  if (!this->initializeScanMatcher(private_nh_)) {
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

  slamProcessor = new hectorslam::HectorSlamProcessor(
    static_cast<float>(p_map_resolution_), p_map_size_, p_map_size_,
    Eigen::Vector2f(p_map_start_x_, p_map_start_y_), p_map_multi_res_levels_,
    callback, hectorDrawings, debugInfoProvider);

  slamProcessor->setUpdateFactorFree(p_update_factor_free_);
  slamProcessor->setUpdateFactorOccupied(p_update_factor_occupied_);
  slamProcessor->setMapUpdateMinDistDiff(p_map_update_distance_threshold_);
  slamProcessor->setMapUpdateMinAngleDiff(p_map_update_angle_threshold_);

  int mapLevels = slamProcessor->getMapLevels();
  mapLevels = 1;

  for (int i = 0; i < mapLevels; ++i)
  {
    mapPubContainer.push_back(MapPublisherContainer());
    slamProcessor->addMapMutex(i, new HectorMapMutex());

    std::string mapTopicStr(mapTopic_);

    if (i != 0)
    {
      mapTopicStr.append("_" + boost::lexical_cast<std::string>(i));
    }

    std::string mapMetaTopicStr(mapTopicStr);
    mapMetaTopicStr.append("_metadata");

    MapPublisherContainer& tmp = mapPubContainer[i];
    tmp.mapPublisher_ = node_.advertise<nav_msgs::OccupancyGrid>(mapTopicStr, 1, true);
    tmp.mapMetadataPublisher_ = node_.advertise<nav_msgs::MapMetaData>(mapMetaTopicStr, 1, true);

    if ( (i == 0) && p_advertise_map_service_)
    {
      tmp.dynamicMapServiceServer_ = node_.advertiseService("dynamic_map", &HectorMappingRos::mapCallback, this);
    }

    setServiceGetMapData(tmp.map_, slamProcessor->getGridMap(i));

    if ( i== 0){
      mapPubContainer[i].mapMetadataPublisher_.publish(mapPubContainer[i].map_.map.info);
    }
  }

  // Initialize services
  reset_map_service_ = node_.advertiseService("reset_map", &HectorMappingRos::resetMapCallback, this);
  restart_hector_service_ = node_.advertiseService("restart_mapping_with_new_pose", &HectorMappingRos::restartHectorCallback, this);
  toggle_scan_processing_service_ = node_.advertiseService("pause_mapping", &HectorMappingRos::pauseMapCallback, this);

  ROS_INFO("HectorSM p_base_frame_: %s", p_base_frame_.c_str());
  ROS_INFO("HectorSM p_map_frame_: %s", p_map_frame_.c_str());
  ROS_INFO("HectorSM p_odom_frame_: %s", p_odom_frame_.c_str());
  ROS_INFO("HectorSM p_scan_topic_: %s", p_scan_topic_.c_str());
  ROS_INFO("HectorSM p_use_tf_scan_transformation_: %s", p_use_tf_scan_transformation_ ? ("true") : ("false"));
  ROS_INFO("HectorSM p_pub_map_odom_transform_: %s", p_pub_map_odom_transform_ ? ("true") : ("false"));
  ROS_INFO("HectorSM p_scan_subscriber_queue_size_: %d", p_scan_subscriber_queue_size_);
  ROS_INFO("HectorSM p_map_pub_period_: %f", p_map_pub_period_);
  ROS_INFO("HectorSM p_update_factor_free_: %f", p_update_factor_free_);
  ROS_INFO("HectorSM p_update_factor_occupied_: %f", p_update_factor_occupied_);
  ROS_INFO("HectorSM p_map_update_distance_threshold_: %f ", p_map_update_distance_threshold_);
  ROS_INFO("HectorSM p_map_update_angle_threshold_: %f", p_map_update_angle_threshold_);
  ROS_INFO("HectorSM p_laser_z_min_value_: %f", p_laser_z_min_value_);
  ROS_INFO("HectorSM p_laser_z_max_value_: %f", p_laser_z_max_value_);

  scanSubscriber_ = node_.subscribe(p_scan_topic_, p_scan_subscriber_queue_size_, &HectorMappingRos::scanCallback, this);
  sysMsgSubscriber_ = node_.subscribe(p_sys_msg_topic_, 2, &HectorMappingRos::sysMsgCallback, this);

  poseUpdatePublisher_ = node_.advertise<geometry_msgs::PoseWithCovarianceStamped>(p_pose_update_topic_, 1, false);
  posePublisher_ = node_.advertise<geometry_msgs::PoseStamped>("slam_out_pose", 1, false);

  scan_point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud>("slam_cloud",1,false);

  tfB_ = new tf::TransformBroadcaster();
  ROS_ASSERT(tfB_);

  /*
  bool p_use_static_map_ = false;

  if (p_use_static_map_){
    mapSubscriber_ = node_.subscribe(mapTopic_, 1, &HectorMappingRos::staticMapCallback, this);
  }
  */

  initial_pose_sub_ = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(node_, "initialpose", 2);
  initial_pose_filter_ = new tf::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(*initial_pose_sub_, tf_, p_map_frame_, 2);
  initial_pose_filter_->registerCallback(boost::bind(&HectorMappingRos::initialPoseCallback, this, _1));


  map__publish_thread_ = new boost::thread(boost::bind(&HectorMappingRos::publishMapLoop, this, p_map_pub_period_));

  map_to_odom_.setIdentity();

  lastMapPublishTime = ros::Time(0,0);
}

HectorMappingRos::~HectorMappingRos()
{
  delete slamProcessor;

  if (hectorDrawings)
    delete hectorDrawings;

  if (debugInfoProvider)
    delete debugInfoProvider;

  if (tfB_)
    delete tfB_;

  if(map__publish_thread_)
    delete map__publish_thread_;
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
        nh, this->hectorDrawings, this->debugInfoProvider);
      return this->mDefaultScanMatcher != nullptr;

    case ScanMatcherOption::GaussNewton:
      this->mGaussNewtonScanMatcher = ScanMatcherGaussNewton::Create(
        nh, this->hectorDrawings, this->debugInfoProvider);
      return this->mGaussNewtonScanMatcher != nullptr;

    case ScanMatcherOption::Correlative:
      this->mCorrelativeScanMatcher = ScanMatcherCorrelative::Create(
        nh, this->hectorDrawings, this->debugInfoProvider);
      return this->mCorrelativeScanMatcher != nullptr;

    case ScanMatcherOption::CorrelativeFPGA:
      this->mFPGAScanMatcher = ScanMatcherFPGA::Create(
        nh, this->hectorDrawings, this->debugInfoProvider);
      return this->mFPGAScanMatcher != nullptr;

    case ScanMatcherOption::GaussNewtonAfterCorrelative:
      this->mCorrelativeScanMatcher = ScanMatcherCorrelative::Create(
        nh, this->hectorDrawings, this->debugInfoProvider);
      this->mGaussNewtonScanMatcher = ScanMatcherGaussNewton::Create(
        nh, this->hectorDrawings, this->debugInfoProvider);
      return this->mCorrelativeScanMatcher != nullptr &&
             this->mGaussNewtonScanMatcher != nullptr;

    case ScanMatcherOption::GaussNewtonAfterCorrelativeFPGA:
      this->mFPGAScanMatcher = ScanMatcherFPGA::Create(
        nh, this->hectorDrawings, this->debugInfoProvider);
      this->mGaussNewtonScanMatcher = ScanMatcherGaussNewton::Create(
        nh, this->hectorDrawings, this->debugInfoProvider);
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
  if (this->pause_scan_processing_)
    return;

  // Initialize the metrics information
  this->mMetricsMsg = hector_mapping::HectorMappingMetrics();
  // Measure the time for processing the current frame (scan)
  Timer outerTimer;

  if (++this->mNumOfProcessedScans % 10 == 0)
    ROS_INFO("Processing the frame: %d", this->mNumOfProcessedScans);

  if (this->hectorDrawings != nullptr)
    this->hectorDrawings->setTime(scan.header.stamp);

  ros::WallTime start_time = ros::WallTime::now();

  if (!this->p_use_tf_scan_transformation_) {
    // Measure the time for preprocessing the current scan
    Timer timer;
    // If we are not using the tf tree to find the transform between the
    // base frame and laser frame, then just convert the laser scan to our data
    // container and process the update based on our last pose estimate
    hectorslam::DataContainer scanContainer;
    this->rosLaserScanToDataContainer(scan, scanContainer,
      this->slamProcessor->getScaleToMap());
    this->mMetricsMsg.scan_preprocessing_time.fromNSec(
      timer.ElapsedNanoseconds());

    this->slamProcessor->update(scanContainer,
      this->slamProcessor->getLastScanMatchPose(), &this->mMetricsMsg);
  } else {
    // If we are using the tf tree to find the transform between the
    // base frame and laser frame, let's get that transform
    tf::StampedTransform laser_transform;
    if (this->tf_.waitForTransform(this->p_base_frame_, scan.header.frame_id,
        scan.header.stamp, ros::Duration(0.5))) {
      this->tf_.lookupTransform(this->p_base_frame_, scan.header.frame_id,
        scan.header.stamp, laser_transform);
    } else {
      ROS_INFO("lookupTransform %s to %s timed out. "
               "Could not transform laser scan into base_frame.",
               this->p_base_frame_.c_str(), scan.header.frame_id.c_str());
      return;
    }

    // Measure the time for preprocessing the current scan
    Timer timer;

    // Convert the laser scan to point cloud
    this->projector_.projectLaser(scan, this->laser_point_cloud_, 30.0);

    // Publish the point cloud if there are any subscribers
    if (this->scan_point_cloud_publisher_.getNumSubscribers() > 0)
      this->scan_point_cloud_publisher_.publish(this->laser_point_cloud_);

    // Convert the point cloud to our data container
    hectorslam::DataContainer scanContainer;
    this->rosPointCloudToDataContainer(
      this->laser_point_cloud_, laser_transform, scanContainer,
      this->slamProcessor->getScaleToMap());
    this->mMetricsMsg.scan_preprocessing_time.fromNSec(
      timer.ElapsedNanoseconds());

    // Now let's choose the initial pose estimate for our slam process update
    Eigen::Vector3f start_estimate { Eigen::Vector3f::Zero() };
    if (this->initial_pose_set_) {
      // User has requested a pose reset
      this->initial_pose_set_ = false;
      start_estimate = this->initial_pose_;
    } else if (this->p_use_tf_pose_start_estimate_) {
      // Initial pose estimate comes from the tf tree
      try {
        tf::StampedTransform stamped_pose;

        this->tf_.waitForTransform(this->p_map_frame_, this->p_base_frame_,
          scan.header.stamp, ros::Duration(0.5));
        this->tf_.lookupTransform(this->p_map_frame_, this->p_base_frame_,
          scan.header.stamp, stamped_pose);

        start_estimate[0] = stamped_pose.getOrigin().getX();
        start_estimate[1] = stamped_pose.getOrigin().getY();
        start_estimate[2] = tf::getYaw(stamped_pose.getRotation());
      } catch(tf::TransformException e) {
        ROS_ERROR("Transform from %s to %s failed\n",
                  this->p_map_frame_.c_str(), this->p_base_frame_.c_str());
        start_estimate = this->slamProcessor->getLastScanMatchPose();
      }
    } else {
      // If none of the above, initial pose is simply the last estimated pose
      start_estimate = this->slamProcessor->getLastScanMatchPose();
    }

    // If "p_map_with_known_poses_" is enabled, we assume that start_estimate
    // is precise and doesn't need to be refined
    if (this->p_map_with_known_poses_)
      this->slamProcessor->update(scanContainer, start_estimate,
        &this->mMetricsMsg, true);
    else
      this->slamProcessor->update(scanContainer, start_estimate,
        &this->mMetricsMsg);
  }

  // If the debug flag "p_timing_output_" is enabled, print how long this last
  // iteration took
  if (this->p_timing_output_) {
    ros::WallDuration duration = ros::WallTime::now() - start_time;
    ROS_INFO("HectorSLAM Iter took: %f milliseconds",
             duration.toSec() * 1000.0f);
  }

  // If we're just building a map with known poses, we're finished now.
  // Code below this point publishes the localization results.
  if (this->p_map_with_known_poses_)
    return;

  Timer timer;

  this->poseInfoContainer_.update(
    this->slamProcessor->getLastScanMatchPose(),
    this->slamProcessor->getLastScanMatchCovariance(),
    scan.header.stamp, this->p_map_frame_);

  // Publish pose with and without covariances
  this->poseUpdatePublisher_.publish(
    this->poseInfoContainer_.getPoseWithCovarianceStamped());
  this->posePublisher_.publish(this->poseInfoContainer_.getPoseStamped());

  // Publish odometry if enabled
  if (this->p_pub_odometry_) {
    const auto& poseAndCovariance =
      this->poseInfoContainer_.getPoseWithCovarianceStamped();
    nav_msgs::Odometry odometry;
    odometry.pose = poseAndCovariance.pose;
    odometry.header = poseAndCovariance.header;
    odometry.child_frame_id = this->p_base_frame_;
    this->odometryPublisher_.publish(odometry);
  }

  // Publish the map->odom transform if enabled
  if (this->p_pub_map_odom_transform_) {
    tf::StampedTransform odom_to_base;
    try {
      this->tf_.waitForTransform(this->p_odom_frame_, this->p_base_frame_,
        scan.header.stamp, ros::Duration(0.5));
      this->tf_.lookupTransform(this->p_odom_frame_, this->p_base_frame_,
        scan.header.stamp, odom_to_base);
    } catch (tf::TransformException e) {
      ROS_ERROR("Transform failed during publishing of map_odom transform: %s",
                e.what());
      odom_to_base.setIdentity();
    }

    this->map_to_odom_ = this->poseInfoContainer_.getTfTransform()
      * odom_to_base.inverse();
    this->tfB_->sendTransform(tf::StampedTransform(
      this->map_to_odom_, scan.header.stamp,
      this->p_map_frame_, this->p_odom_frame_));
  }

  // Publish the transform from map to estimated pose (if enabled)
  if (this->p_pub_map_scanmatch_transform_)
    this->tfB_->sendTransform(tf::StampedTransform(
      this->poseInfoContainer_.getTfTransform(), scan.header.stamp,
      this->p_map_frame_, this->p_tf_map_scanmatch_transform_frame_name_));

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
    this->slamProcessor->reset();
  } else if (sysMsg.data == "shutdown") {
    ROS_INFO("Hector SLAM node is shutting down");
    ros::requestShutdown();
  }
}

bool HectorMappingRos::mapCallback(nav_msgs::GetMap::Request  &req,
                                   nav_msgs::GetMap::Response &res)
{
  ROS_INFO("HectorSM Map service called");
  res = mapPubContainer[0].map_;
  return true;
}

bool HectorMappingRos::resetMapCallback(std_srvs::Trigger::Request  &req,
                                        std_srvs::Trigger::Response &res)
{
  ROS_INFO("HectorSM Reset map service called");
  slamProcessor->reset();
  return true;
}

bool HectorMappingRos::restartHectorCallback(hector_mapping::ResetMapping::Request  &req,
                                             hector_mapping::ResetMapping::Response &res)
{
  // Reset map
  ROS_INFO("HectorSM Reset map");
  slamProcessor->reset();

  // Reset pose
  this->resetPose(req.initial_pose);

  // Unpause node (in case it is paused)
  this->toggleMappingPause(false);

  // Return success
  return true;
}

bool HectorMappingRos::pauseMapCallback(std_srvs::SetBool::Request  &req,
                                        std_srvs::SetBool::Response &res)
{
  this->toggleMappingPause(req.data);
  res.success = true;
  return true;
}

void HectorMappingRos::publishMap(MapPublisherContainer& mapPublisher, const hectorslam::GridMap& gridMap, ros::Time timestamp, MapLockerInterface* mapMutex)
{
  nav_msgs::GetMap::Response& map_ (mapPublisher.map_);

  //only update map if it changed
  if (lastGetMapUpdateIndex != gridMap.getUpdateIndex())
  {

    int sizeX = gridMap.getSizeX();
    int sizeY = gridMap.getSizeY();

    int size = sizeX * sizeY;

    std::vector<int8_t>& data = map_.map.data;

    //std::vector contents are guaranteed to be contiguous, use memset to set all to unknown to save time in loop
    memset(&data[0], -1, sizeof(int8_t) * size);

    if (mapMutex)
    {
      mapMutex->lockMap();
    }

    for(int i=0; i < size; ++i)
    {
      if(gridMap.isFree(i))
      {
        data[i] = 0;
      }
      else if (gridMap.isOccupied(i))
      {
        data[i] = 100;
      }
    }

    lastGetMapUpdateIndex = gridMap.getUpdateIndex();

    if (mapMutex)
    {
      mapMutex->unlockMap();
    }
  }

  map_.map.header.stamp = timestamp;

  mapPublisher.mapPublisher_.publish(map_.map);
}

void HectorMappingRos::rosLaserScanToDataContainer(const sensor_msgs::LaserScan& scan, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = scan.ranges.size();

  float angle = scan.angle_min;

  dataContainer.clear();

  dataContainer.setOrigo(Eigen::Vector2f::Zero());

  float maxRangeForContainer = scan.range_max - 0.1f;

  for (size_t i = 0; i < size; ++i)
  {
    float dist = scan.ranges[i];

    if ( (dist > scan.range_min) && (dist < maxRangeForContainer))
    {
      dist *= scaleToMap;
      dataContainer.add(Eigen::Vector2f(cos(angle) * dist, sin(angle) * dist));
    }

    angle += scan.angle_increment;
  }
}

void HectorMappingRos::rosPointCloudToDataContainer(const sensor_msgs::PointCloud& pointCloud, const tf::StampedTransform& laserTransform, hectorslam::DataContainer& dataContainer, float scaleToMap)
{
  size_t size = pointCloud.points.size();
  //ROS_INFO("size: %d", size);

  dataContainer.clear();

  tf::Vector3 laserPos (laserTransform.getOrigin());
  dataContainer.setOrigo(Eigen::Vector2f(laserPos.x(), laserPos.y())*scaleToMap);

  for (size_t i = 0; i < size; ++i)
  {

    const geometry_msgs::Point32& currPoint(pointCloud.points[i]);

    float dist_sqr = currPoint.x*currPoint.x + currPoint.y* currPoint.y;

    if ( (dist_sqr > p_sqr_laser_min_dist_) && (dist_sqr < p_sqr_laser_max_dist_) ){

      if ( (currPoint.x < 0.0f) && (dist_sqr < 0.50f)){
        continue;
      }

      tf::Vector3 pointPosBaseFrame(laserTransform * tf::Vector3(currPoint.x, currPoint.y, currPoint.z));

      float pointPosLaserFrameZ = pointPosBaseFrame.z() - laserPos.z();

      if (pointPosLaserFrameZ > p_laser_z_min_value_ && pointPosLaserFrameZ < p_laser_z_max_value_)
      {
        dataContainer.add(Eigen::Vector2f(pointPosBaseFrame.x(),pointPosBaseFrame.y())*scaleToMap);
      }
    }
  }
}

void HectorMappingRos::setServiceGetMapData(nav_msgs::GetMap::Response& map_, const hectorslam::GridMap& gridMap)
{
  Eigen::Vector2f mapOrigin (gridMap.getWorldCoords(Eigen::Vector2f::Zero()));
  mapOrigin.array() -= gridMap.getCellLength()*0.5f;

  map_.map.info.origin.position.x = mapOrigin.x();
  map_.map.info.origin.position.y = mapOrigin.y();
  map_.map.info.origin.orientation.w = 1.0;

  map_.map.info.resolution = gridMap.getCellLength();

  map_.map.info.width = gridMap.getSizeX();
  map_.map.info.height = gridMap.getSizeY();

  map_.map.header.frame_id = p_map_frame_;
  map_.map.data.resize(map_.map.info.width * map_.map.info.height);
}

/*
void HectorMappingRos::setStaticMapData(const nav_msgs::OccupancyGrid& map)
{
  float cell_length = map.info.resolution;
  Eigen::Vector2f mapOrigin (map.info.origin.position.x + cell_length*0.5f,
                             map.info.origin.position.y + cell_length*0.5f);

  int map_size_x = map.info.width;
  int map_size_y = map.info.height;

  slamProcessor = new hectorslam::HectorSlamProcessor(cell_length, map_size_x, map_size_y, Eigen::Vector2f(0.0f, 0.0f), 1, hectorDrawings, debugInfoProvider);
}
*/


void HectorMappingRos::publishMapLoop(double map_pub_period)
{
  ros::Rate r(1.0 / map_pub_period);
  while(ros::ok())
  {
    //ros::WallTime t1 = ros::WallTime::now();
    ros::Time mapTime (ros::Time::now());
    //publishMap(mapPubContainer[2],slamProcessor->getGridMap(2), mapTime);
    //publishMap(mapPubContainer[1],slamProcessor->getGridMap(1), mapTime);
    publishMap(mapPubContainer[0],slamProcessor->getGridMap(0), mapTime, slamProcessor->getMapMutex(0));

    //ros::WallDuration t2 = ros::WallTime::now() - t1;

    //std::cout << "time s: " << t2.toSec();
    //ROS_INFO("HectorSM ms: %4.2f", t2.toSec()*1000.0f);

    r.sleep();
  }
}

void HectorMappingRos::staticMapCallback(const nav_msgs::OccupancyGrid& map)
{

}

void HectorMappingRos::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  this->resetPose(msg->pose.pose);
}

void HectorMappingRos::toggleMappingPause(bool pause)
{
  // Pause/unpause
  if (pause && !pause_scan_processing_)
  {
    ROS_INFO("[HectorSM]: Mapping paused");
  }
  else if (!pause && pause_scan_processing_)
  {
    ROS_INFO("[HectorSM]: Mapping no longer paused");
  }
  pause_scan_processing_ = pause;
}

void HectorMappingRos::resetPose(const geometry_msgs::Pose &pose)
{
  initial_pose_set_ = true;
  initial_pose_ = Eigen::Vector3f(pose.position.x, pose.position.y, util::getYawFromQuat(pose.orientation));
  ROS_INFO("[HectorSM]: Setting initial pose with world coords x: %f y: %f yaw: %f",
           initial_pose_[0], initial_pose_[1], initial_pose_[2]);
}
