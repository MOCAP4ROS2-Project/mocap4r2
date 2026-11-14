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

#include <algorithm>
#include <string>

#include "mocap4r2_control/frame_stats.hpp"

namespace mocap4r2_control
{

void
FrameStats::reset(double now_s)
{
  std::lock_guard<std::mutex> lock(mutex_);
  last_frame_.reset();
  window_start_ = now_s;
  frames_ = dropped_ = total_frames_ = total_dropped_ = 0;
  latency_sum_ = latency_max_ = 0.0;
  tracking_.clear();
}

void
FrameStats::add_frame(uint64_t frame_number, double latency_s)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (last_frame_ && frame_number == *last_frame_) {
    return;
  }
  // Gaps count as drops; a lower number means the mocap software restarted.
  if (last_frame_ && frame_number > *last_frame_ + 1) {
    const uint64_t gap = frame_number - *last_frame_ - 1;
    dropped_ += gap;
    total_dropped_ += gap;
  }
  last_frame_ = frame_number;
  frames_++;
  total_frames_++;
  latency_sum_ += latency_s;
  latency_max_ = std::max(latency_max_, latency_s);
}

void
FrameStats::add_tracking(const std::string & name, bool valid)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & t = tracking_[name];
  t.seen++;
  if (valid) {
    t.valid++;
  }
}

FrameStatsSnapshot
FrameStats::take(double now_s)
{
  std::lock_guard<std::mutex> lock(mutex_);
  FrameStatsSnapshot s;
  s.period = std::max(0.0, now_s - window_start_);
  s.frames = frames_;
  s.dropped = dropped_;
  s.rate = s.period > 0.0 ? static_cast<double>(frames_) / s.period : 0.0;
  s.latency_mean = frames_ > 0 ? latency_sum_ / static_cast<double>(frames_) : 0.0;
  s.latency_max = latency_max_;
  s.total_frames = total_frames_;
  s.total_dropped = total_dropped_;
  for (const auto & [name, t] : tracking_) {
    s.tracking_ratio[name] = t.seen > 0 ? static_cast<double>(t.valid) / t.seen : 0.0;
  }

  window_start_ = now_s;
  frames_ = dropped_ = 0;
  latency_sum_ = latency_max_ = 0.0;
  tracking_.clear();
  return s;
}

}  // namespace mocap4r2_control
