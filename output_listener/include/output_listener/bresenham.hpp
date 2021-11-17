
// bresenham.hpp

#ifndef OUTPUT_LISTENER_BRESENHAM_HPP
#define OUTPUT_LISTENER_BRESENHAM_HPP

#include <cmath>
#include <vector>

#include <Eigen/Core>

// Bresenham algorithm
std::vector<Eigen::Vector2i> Bresenham(
  const Eigen::Vector2i& startIdx, const Eigen::Vector2i& endIdx)
{
  std::vector<Eigen::Vector2i> indices;

  int deltaX = endIdx.x() - startIdx.x();
  int deltaY = endIdx.y() - startIdx.y();
  int stepX = (deltaX < 0) ? -1 : 1;
  int stepY = (deltaY < 0) ? -1 : 1;
  int nextX = startIdx.x();
  int nextY = startIdx.y();

  deltaX = std::abs(deltaX * 2);
  deltaY = std::abs(deltaY * 2);

  // Append the start cell index
  indices.emplace_back(nextX, nextY);

  // Bresenham algorithm
  if (deltaX > deltaY) {
    int err = deltaY - deltaX / 2;

    while (nextX != endIdx.x()) {
      if (err >= 0) {
        nextY += stepY;
        err -= deltaX;
      }
      nextX += stepX;
      err += deltaY;
      indices.emplace_back(nextX, nextY);
    }
  } else {
    int err = deltaX - deltaY / 2;

    while (nextY != endIdx.y()) {
      if (err >= 0) {
        nextX += stepX;
        err -= deltaY;
      }
      nextY += stepY;
      err += deltaX;
      indices.emplace_back(nextX, nextY);
    }
  }

  return indices;
}

#endif // OUTPUT_LISTENER_BRESENHAM_HPP
