
// Parameter.hpp

#ifndef HECTOR_SLAM_UTIL_PARAMETER_HPP
#define HECTOR_SLAM_UTIL_PARAMETER_HPP

#include <cstdint>
#include <string>
#include <ros/node_handle.h>

template <typename T>
inline bool GetParamFromNodeHandle(
  ros::NodeHandle& nh, const std::string& key, T& value)
{
  if (!nh.getParam(key, value)) {
    ROS_ERROR("Parameter %s does not exist in the namespace %s",
              key.c_str(), nh.getNamespace().c_str());
    return false;
  }
  return true;
}

inline bool GetAddressFromNodeHandle(
  ros::NodeHandle& nh, const std::string& key, std::uint32_t& address)
{
  std::string addressStr;

  if (!nh.getParam(key, addressStr)) {
    ROS_ERROR("Parameter %s does not exist in the namespace %s",
              key.c_str(), nh.getNamespace().c_str());
    return false;
  }

  // Parse the string as the address value
  address = static_cast<std::uint32_t>(std::stoul(addressStr, nullptr, 0));
  return true;
}

#endif // HECTOR_SLAM_UTIL_PARAMETER_HPP
