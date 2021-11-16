
// HectorMapMutex.h

#ifndef HECTOR_SLAM_HECTOR_MAP_MUTEX_H
#define HECTOR_SLAM_HECTOR_MAP_MUTEX_H

#include <boost/thread/mutex.hpp>

#include "util/MapLockerInterface.h"

class HectorMapMutex : public MapLockerInterface
{
public:
  virtual void lockMap() { this->mapModifyMutex_.lock(); }
  virtual void unlockMap() { this->mapModifyMutex_.unlock(); }

  boost::mutex mapModifyMutex_;
};

#endif // HECTOR_SLAM_HECTOR_MAP_MUTEX_H
