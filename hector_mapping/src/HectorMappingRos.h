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

#ifndef HECTOR_SLAM_HECTOR_MAPPING_ROS_H
#define HECTOR_SLAM_HECTOR_MAPPING_ROS_H

#include <memory>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/message_filter.h>

#include "hector_mapping/GetOccupancyGrids.h"
#include "hector_mapping/ResetMapping.h"
#include "hector_mapping/HectorMappingMetrics.h"

#include "matcher/ScanMatcherCorrelative.hpp"
#include "matcher/ScanMatcherFPGA.hpp"
#include "matcher/ScanMatcherGaussNewton.hpp"
#include "matcher/ScanMatcherOptions.hpp"
#include "matcher/ScanMatcherTypes.hpp"
#include "scan/DataPointContainer.h"
#include "slam_main/HectorSlamProcessor.h"
#include "util/MapLockerInterface.h"

#include "HectorDebugInfoProvider.h"
#include "HectorDrawings.h"
#include "PoseInfoContainer.h"

class MapPublisherContainer
{
public:
  ros::Publisher mapPublisher_;
  ros::Publisher mapMetadataPublisher_;
  nav_msgs::GetMap::Response map_;
  ros::ServiceServer dynamicMapServiceServer_;
};

struct OdometryData
{
  // Default constructor
  OdometryData() : mIsFirstFrame(true),
                   mIsAvailable(false),
                   mLastScanMatchPose(0.0f, 0.0f, 0.0f),
                   mPreviousPose(0.0f, 0.0f, 0.0f),
                   mPose(0.0f, 0.0f, 0.0f),
                   mPreviousTimestamp(0.0),
                   mTimestamp(0.0),
                   mTravelDistance(0.0f),
                   mRotationAngle(0.0f),
                   mElapsedTime(0.0f),
                   mTravelDistanceThreshold(0.0f),
                   mRotationAngleThreshold(0.0f),
                   mElapsedTimeThreshold(0.0f) { }

  // Flag to indicate whether the first scan is processed
  bool mIsFirstFrame;
  // Flag to indicate whether the first odometry data has arrived
  bool mIsAvailable;
  // Odometry pose when the last scan matching is performed
  Eigen::Vector3f mLastScanMatchPose;
  // Odometry pose of the previous odometry message
  Eigen::Vector3f mPreviousPose;
  // Odometry pose of the latest odometry message
  Eigen::Vector3f mPose;
  // Timestamp of the previous odometry message
  ros::Time mPreviousTimestamp;
  // Timestamp of the latest odometry pose
  ros::Time mTimestamp;
  // Travel distance since the last scan matching
  float mTravelDistance;
  // Rotation angle since the last scan matching
  float mRotationAngle;
  // Elapsed time since the last scan matching
  float mElapsedTime;

  // Threshold for the travel distance
  float mTravelDistanceThreshold;
  // Threshold for the rotation angle
  float mRotationAngleThreshold;
  // Threshold for the elapsed time
  float mElapsedTimeThreshold;
};

class HectorMappingRos
{
public:
  HectorMappingRos();
  ~HectorMappingRos();

  bool setupFPGA(ros::NodeHandle& nh);
  bool initializeScanMatcher(ros::NodeHandle& nh);

  Eigen::Vector3f scanMatchCallback(
    const Eigen::Vector3f& initialWorldPose,
    hectorslam::GridMapUtil& gridMapUtil,
    const hectorslam::DataContainer& dataContainer,
    Eigen::Matrix3f& covMatrix, const int mapIndex);

  void odomCallback(const nav_msgs::Odometry& odometry);
  void scanCallback(const sensor_msgs::LaserScan& scan);
  void sysMsgCallback(const std_msgs::String& string);

  bool occupancyGridsCallback(
    hector_mapping::GetOccupancyGrids::Request& request,
    hector_mapping::GetOccupancyGrids::Response& response);
  bool mapCallback(nav_msgs::GetMap::Request &req,
                   nav_msgs::GetMap::Response &res);
  bool resetMapCallback(std_srvs::Trigger::Request &req,
                        std_srvs::Trigger::Response &res);
  bool restartHectorCallback(hector_mapping::ResetMapping::Request &req,
                             hector_mapping::ResetMapping::Response &res);
  bool pauseMapCallback(std_srvs::SetBool::Request &req,
                        std_srvs::SetBool::Response &res);

  void publishMap(MapPublisherContainer& map_,
                  const hectorslam::GridMap& gridMap,
                  ros::Time timestamp,
                  MapLockerInterface* mapMutex = nullptr);

  void rosLaserScanToDataContainer(const sensor_msgs::LaserScan& scan,
                                   hectorslam::DataContainer& dataContainer,
                                   float scaleToMap);
  void rosPointCloudToDataContainer(const sensor_msgs::PointCloud& pointCloud,
                                    const tf::StampedTransform& laserTransform,
                                    hectorslam::DataContainer& dataContainer,
                                    float scaleToMap);

  void setServiceGetMapData(nav_msgs::GetMap::Response& map_,
                            const hectorslam::GridMap& gridMap);

  void publishTransformLoop(double p_transform_pub_period_);
  void publishMapLoop(double p_map_pub_period_);
  void publishTransform();

  void initialPoseCallback(
    const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);

  // Internal mapping management functions
  void toggleMappingPause(bool pause);
  void resetPose(const geometry_msgs::Pose &pose);

  // void setStaticMapData(const nav_msgs::OccupancyGrid& map);

protected:
  std::unique_ptr<HectorDebugInfoProvider> mDebugInfoProvider;
  std::unique_ptr<HectorDrawings> mHectorDrawings;

  int mLastGetMapUpdateIndex;

  ros::NodeHandle mNode;

  ros::Subscriber mOdomSubscriber;
  ros::Subscriber mScanSubscriber;
  ros::Subscriber mSysMsgSubscriber;

  std::unique_ptr<message_filters::Subscriber<
    geometry_msgs::PoseWithCovarianceStamped>> mInitialPoseSubscriber;
  std::unique_ptr<tf::MessageFilter<
    geometry_msgs::PoseWithCovarianceStamped>> mInitialPoseFilter;

  ros::Publisher mPosePublisher;
  ros::Publisher mPoseUpdatePublisher;
  ros::Publisher mOdometryPublisher;
  ros::Publisher mScanPointCloudPublisher;

  ros::ServiceServer mSrvOccupancyGrids;
  ros::ServiceServer mResetMapService;
  ros::ServiceServer mRestartHectorService;
  ros::ServiceServer mToggleScanProcessingService;

  std::vector<MapPublisherContainer> mMapPubContainer;

  tf::TransformListener mTf;
  std::unique_ptr<tf::TransformBroadcaster> mTfBroadcaster;

  std::unique_ptr<boost::thread> mMapPublishThread;

  std::unique_ptr<hectorslam::HectorSlamProcessor> mSlamProcessor;

  int mNumOfProcessedScans;
  hector_mapping::HectorMappingMetrics mMetricsMsg;
  ros::Publisher mMetricsPublisher;

  hectorslam::ScanMatcherOption mScanMatcherOption;
  std::unique_ptr<hectorslam::DefaultScanMatcher> mDefaultScanMatcher;
  std::unique_ptr<hectorslam::ScanMatcherCorrelative> mCorrelativeScanMatcher;
  std::unique_ptr<hectorslam::ScanMatcherFPGA> mFPGAScanMatcher;
  std::unique_ptr<hectorslam::ScanMatcherGaussNewton> mGaussNewtonScanMatcher;

  PoseInfoContainer mPoseInfoContainer;

  bool mInitialPoseSet;
  Eigen::Vector3f mInitialPose;

  OdometryData mOdometry;

  bool mPauseScanProcessing;

  //
  // Parameters
  //

  std::string mBaseFrame;
  std::string mMapFrame;
  std::string mOdomFrame;

  // Parameters related to publishing the scanmatcher pose directly via tf
  bool mPubMapToScanMatchTransform;
  std::string mMapToScanMatchTransformFrameName;

  std::string mOdomTopic;
  std::string mScanTopic;
  std::string mSysMsgTopic;
  std::string mPoseUpdateTopic;

  bool mPubDrawings;
  bool mPubDebugOutput;
  bool mPubMapToOdomTransform;
  bool mPubOdometry;
  bool mAdvertiseMapService;
  int mScanSubscriberQueueSize;

  double mUpdateFactorFree;
  double mUpdateFactorOccupied;
  double mMapUpdateDistanceThreshold;
  double mMapUpdateAngleThreshold;

  double mMapResolution;
  int mMapSize;
  double mMapStartX;
  double mMapStartY;
  int mMapMultiResLevels;

  double mMapPubPeriod;

  bool mUseTfScanTransformation;
  bool mUseTfPoseStartEstimate;
  bool mUseOdometryPose;
  bool mMapWithKnownPoses;
  bool mTimingOutput;

  float mSquaredLaserMinDist;
  float mSquaredLaserMaxDist;
  float mLaserZMinValue;
  float mLaserZMaxValue;
};

#endif // HECTOR_SLAM_HECTOR_MAPPING_ROS_H
