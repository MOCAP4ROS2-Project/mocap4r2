// Copyright 2026 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MOCAP4R2_CONTROL__FRAME_STATS_HPP_
#define MOCAP4R2_CONTROL__FRAME_STATS_HPP_

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>

namespace mocap4r2_control
{

// Statistics of one window (between two calls to FrameStats::take).
struct FrameStatsSnapshot
{
  double period{0.0};           // seconds covered by the window
  uint64_t frames{0};           // frames received in the window
  uint64_t dropped{0};          // frames missing (gaps in frame numbers) in the window
  double rate{0.0};             // frames / period, Hz
  double latency_mean{0.0};     // seconds, 0 if no frames
  double latency_max{0.0};      // seconds
  uint64_t total_frames{0};     // since reset
  uint64_t total_dropped{0};    // since reset
  std::map<std::string, double> tracking_ratio;  // body name -> valid / seen in the window
};

// Thread-safe frame statistics; fed from SDK threads, read by diagnostics.
class FrameStats
{
public:
  void reset(double now_s);

  // Count one frame. Repeated frame numbers are ignored; a lower one is a restart.
  void add_frame(uint64_t frame_number, double latency_s);

  // Count one observation of a tracked body in the current frame.
  void add_tracking(const std::string & name, bool valid);

  // Return the current window and start a new one.
  FrameStatsSnapshot take(double now_s);

private:
  struct Tracking
  {
    uint64_t valid{0};
    uint64_t seen{0};
  };

  std::mutex mutex_;
  std::optional<uint64_t> last_frame_;
  double window_start_{0.0};
  uint64_t frames_{0};
  uint64_t dropped_{0};
  double latency_sum_{0.0};
  double latency_max_{0.0};
  uint64_t total_frames_{0};
  uint64_t total_dropped_{0};
  std::map<std::string, Tracking> tracking_;
};

}  // namespace mocap4r2_control

#endif  // MOCAP4R2_CONTROL__FRAME_STATS_HPP_
