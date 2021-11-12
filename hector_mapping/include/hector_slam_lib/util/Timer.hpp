
// Timer.hpp

#ifndef HECTOR_SLAM_UTIL_TIMER_HPP
#define HECTOR_SLAM_UTIL_TIMER_HPP

#include <chrono>
#include <cstdint>

#include <boost/timer/timer.hpp>

namespace hectorslam {

struct Timer
{
  // Constructor (internal timer is automatically started)
  Timer() = default;
  // Destructor
  ~Timer() = default;

  // Convert nanoseconds to milliseconds
  inline std::int64_t ToMilliseconds(const std::int_least64_t nanoSec) const
  { return std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::nanoseconds(nanoSec)).count(); }

  // Convert nanoseconds to microseconds
  inline std::int64_t ToMicroseconds(const std::int_least64_t nanoSec) const
  { return std::chrono::duration_cast<std::chrono::microseconds>(
    std::chrono::nanoseconds(nanoSec)).count(); }

  // Get the elapsed time in microseconds
  inline std::int64_t ElapsedMicroseconds() const
  { return this->ToMicroseconds(this->mTimer.elapsed().wall); }
  // Get the elapsed time in milliseconds
  inline std::int64_t ElapsedMilliseconds() const
  { return this->ToMilliseconds(this->mTimer.elapsed().wall); }
  // Get the elapsed time in nanoseconds
  inline std::int64_t ElapsedNanoseconds() const
  { return this->mTimer.elapsed().wall; }

  // Check if the timer is stopped
  inline bool IsStopped() const { return this->mTimer.is_stopped(); }
  // Stop the timer
  inline void Stop() { this->mTimer.stop(); }
  // Start the timer (timer restarts from zero)
  inline void Start() { this->mTimer.start(); }
  // Resume the timer
  inline void Resume() { this->mTimer.resume(); }

  // Internal timer instance
  boost::timer::cpu_timer mTimer;
};

} // namespace hectorslam

#endif // HECTOR_SLAM_UTIL_TIMER_HPP
