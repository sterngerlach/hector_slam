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

#ifndef HECTOR_SLAM_HECTOR_DRAWINGS_H
#define HECTOR_SLAM_HECTOR_DRAWINGS_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>

#include "util/DrawInterface.h"
#include "util/UtilFunctions.h"

class HectorDrawings : public DrawInterface
{
public:
  HectorDrawings()
  {
    this->idCounter = 0;

    ros::NodeHandle nh_;

    this->markerPublisher_ = nh_.advertise<
      visualization_msgs::Marker>("visualization_marker", 1, true);
    this->markerArrayPublisher_ = nh_.advertise<
      visualization_msgs::MarkerArray>("visualization_marker_array", 1, true);

    this->tempMarker.header.frame_id = "map";
    this->tempMarker.ns = "slam";

    this->setScale(1.0);
    this->setColor(1.0, 1.0, 1.0);

    this->tempMarker.action = visualization_msgs::Marker::ADD;
  }

  virtual void drawPoint(const Eigen::Vector2f& pointWorldFrame)
  {
    this->tempMarker.id = this->idCounter++;

    this->tempMarker.pose.position.x = pointWorldFrame.x();
    this->tempMarker.pose.position.y = pointWorldFrame.y();

    this->tempMarker.pose.orientation.w = 0.0;
    this->tempMarker.pose.orientation.z = 0.0;
    this->tempMarker.type = visualization_msgs::Marker::CUBE;

    // this->markerPublisher_.publish(tempMarker);

    this->markerArray.markers.push_back(tempMarker);
  }

  virtual void drawArrow(const Eigen::Vector3f& poseWorld)
  {
    this->tempMarker.id = this->idCounter++;

    this->tempMarker.pose.position.x = poseWorld.x();
    this->tempMarker.pose.position.y = poseWorld.y();

    this->tempMarker.pose.orientation.w = std::cos(poseWorld.z() * 0.5f);
    this->tempMarker.pose.orientation.z = std::sin(poseWorld.z() * 0.5f);

    this->tempMarker.type = visualization_msgs::Marker::ARROW;

    // this->markerPublisher_.publish(tempMarker);

    this->markerArray.markers.push_back(tempMarker);
  }

  virtual void drawCovariance(const Eigen::Vector2f& mean,
                              const Eigen::Matrix2f& covMatrix)
  {
    this->tempMarker.pose.position.x = mean[0];
    this->tempMarker.pose.position.y = mean[1];

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eig { covMatrix };

    const Eigen::Vector2f& eigValues = eig.eigenvalues();
    const Eigen::Matrix2f& eigVectors = eig.eigenvectors();

    const float angle = std::atan2(eigVectors(1, 0), eigVectors(0, 0));

    this->tempMarker.type = visualization_msgs::Marker::CYLINDER;

    const double lengthMajor = std::sqrt(eigValues[0]);
    const double lengthMinor = std::sqrt(eigValues[1]);

    this->tempMarker.scale.x = lengthMajor;
    this->tempMarker.scale.y = lengthMinor;
    this->tempMarker.scale.z = 0.001;

    this->tempMarker.pose.orientation.w = std::cos(angle * 0.5);
    this->tempMarker.pose.orientation.z = std::sin(angle * 0.5);

    this->tempMarker.id = this->idCounter++;
    this->markerArray.markers.push_back(this->tempMarker);

    // this->drawLine(Eigen::Vector3f(0, 0, 0),
    //   Eigen::Vector3f(lengthMajor, 0, 0));
    // this->drawLine(Eigen::Vector3f(0, 0, 0),
    //   Eigen::Vector3f(0, lengthMinor, 0));

    // glScalef(lengthMajor, lengthMinor, 0);
    // glCallList(dlCircle);
    // this->popCS();
  }

  virtual void setScale(double scale)
  {
    this->tempMarker.scale.x = scale;
    this->tempMarker.scale.y = scale;
    this->tempMarker.scale.z = scale;
  }

  virtual void setColor(double r, double g, double b, double a = 1.0)
  {
    this->tempMarker.color.r = r;
    this->tempMarker.color.g = g;
    this->tempMarker.color.b = b;
    this->tempMarker.color.a = a;
  }

  virtual void sendAndResetData()
  {
    this->markerArrayPublisher_.publish(this->markerArray);
    this->markerArray.markers.clear();
    this->idCounter = 0;
  }

  void setTime(const ros::Time& time)
  {
    this->tempMarker.header.stamp = time;
  }

  ros::Publisher markerPublisher_;
  ros::Publisher markerArrayPublisher_;

  visualization_msgs::Marker tempMarker;
  visualization_msgs::MarkerArray markerArray;

  int idCounter;
};

#endif // HECTOR_SLAM_HECTOR_DRAWINGS_H
