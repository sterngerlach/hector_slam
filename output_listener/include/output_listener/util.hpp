
// util.hpp

#ifndef OUTPUT_LISTENER_UTIL_HPP
#define OUTPUT_LISTENER_UTIL_HPP

#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>

#include <boost/array.hpp>

#include <Eigen/Geometry>

// Convert `geometry_msgs::Quaternion` to Euler angles
Eigen::Vector3d ToEulerAngles(const geometry_msgs::Quaternion& quatMsg)
{
  const tf::Quaternion quat { quatMsg.x, quatMsg.y, quatMsg.z, quatMsg.w };
  const tf::Matrix3x3 mat { quat };
  Eigen::Vector3d eulerAngles;
  mat.getRPY(eulerAngles.x(), eulerAngles.y(), eulerAngles.z());
  return eulerAngles;
}

// Extract 2D pose covariance from the 3D pose covariance
Eigen::Matrix3d ToPose2DCovariance(const double* poseCov3D)
{
  Eigen::Matrix3d poseCov2D;
  poseCov2D << poseCov3D[0],  poseCov3D[1],  poseCov3D[5],
               poseCov3D[6],  poseCov3D[7],  poseCov3D[11],
               poseCov3D[30], poseCov3D[31], poseCov3D[35];
  return poseCov2D;
}

// Convert array to std::string
template <typename T>
std::string ArrayToString(const T* array,
                          const std::size_t numOfElements,
                          const int precision = -1)
{
  std::ostringstream strStream;

  if (precision != -1)
    strStream << std::fixed << std::setprecision(precision);

  for (std::size_t i = 0; i < numOfElements; ++i)
    if (i == numOfElements - 1)
      strStream << array[i];
    else
      strStream << array[i] << ' ';

  return strStream.str();
}

// Convert std::vector to std::string
template <typename T>
std::string VecToString(const std::vector<T>& vec,
                        const int precision = -1)
{
  return ArrayToString(vec.data(), vec.size(), precision);
}

// Convert boost::array<> to std::string
template <typename T, std::size_t N>
std::string BoostArrayToString(const boost::array<T, N>& array,
                               const int precision = -1)
{
  return ArrayToString(array.data(), array.size(), precision);
}

// Convert 2D pose to std::string
std::string Pose2DToString(const boost::array<float, 3>& pose)
{
  return ArrayToString(pose.data(), pose.size(),
                       std::numeric_limits<float>::digits10);
}

// Convert 2D pose covariance to std::string
std::string Pose2DCovarianceToString(const boost::array<float, 9>& poseCov)
{
  // Get only 6 elements since the covariance is symmetric
  boost::array<float, 6> elements = { poseCov[0], poseCov[1], poseCov[2],
                                      poseCov[4], poseCov[5], poseCov[8] };
  return ArrayToString(elements.data(), elements.size(),
                       std::numeric_limits<float>::digits10);
}

#endif // OUTPUT_LISTENER_UTIL_HPP
