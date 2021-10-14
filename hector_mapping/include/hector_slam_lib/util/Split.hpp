
/* Split.hpp */

#ifndef HECTOR_SLAM_UTIL_SPLIT_HPP
#define HECTOR_SLAM_UTIL_SPLIT_HPP

#include <string>
#include <vector>

namespace hectorslam {

/* Split a given string with a delimiter string */
std::vector<std::string> Split(const std::string& str,
                               const std::string& delimiter)
{
  const auto delimiterLength = delimiter.size();

  if (delimiterLength == 0)
    return std::vector<std::string> { str };

  std::vector<std::string> elementVec;
  auto offsetPos = std::string::size_type(0);

  while (true) {
    const auto foundPos = str.find(delimiter, offsetPos);

    if (foundPos == std::string::npos) {
      elementVec.push_back(str.substr(offsetPos));
      break;
    } else {
      elementVec.push_back(str.substr(offsetPos, foundPos - offsetPos));
      offsetPos = foundPos + delimiterLength;
    }
  }

  return elementVec;
}

/* Split a given string with a delimiter character */
std::vector<std::string> Split(const std::string& str,
                               const char delimiter)
{
  std::vector<std::string> elementVec;
  auto offsetPos = std::string::size_type(0);

  while (true) {
    const auto foundPos = str.find(delimiter, offsetPos);

    if (foundPos == std::string::npos) {
      elementVec.push_back(str.substr(offsetPos));
      break;
    } else {
      elementVec.push_back(str.substr(offsetPos, foundPos - offsetPos));
      offsetPos = foundPos + 1;
    }
  }

  return elementVec;
}

} /* namespace hectorslam */

#endif /* HECTOR_SLAM_UTIL_SPLIT_HPP */
