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

#ifndef _hectormapproccontainer_h__
#define _hectormapproccontainer_h__

#include <memory>

#include "../map/GridMap.h"
#include "../map/OccGridMapUtilConfig.h"
#include "../util/MapLockerInterface.h"

class GridMap;
class ConcreteOccGridMapUtil;
class DataContainer;

namespace hectorslam{

class MapProcContainer
{
public:
  // Type declarations for convenience
  using GridMapUtil = OccGridMapUtilConfig<GridMap>;

  MapProcContainer(
    std::unique_ptr<GridMap> gridMapIn,
    std::unique_ptr<GridMapUtil> gridMapUtilIn) :
    gridMap(std::move(gridMapIn)),
    gridMapUtil(std::move(gridMapUtilIn)),
    mapMutex(nullptr) { }

  virtual ~MapProcContainer() { }

  // Copy constructor (deleted)
  MapProcContainer(const MapProcContainer&) = delete;
  // Copy assignment operator (deleted)
  MapProcContainer& operator=(const MapProcContainer&) = delete;
  // Move constructor
  MapProcContainer(MapProcContainer&&) = default;
  // Move assignment operator
  MapProcContainer& operator=(MapProcContainer&&) = default;

  std::unique_ptr<GridMap> gridMap;
  std::unique_ptr<GridMapUtil> gridMapUtil;
  std::unique_ptr<MapLockerInterface> mapMutex;
};

}

#endif
