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

#include <gtest/gtest.h>

#include "mocap4r2_control/frame_stats.hpp"

using mocap4r2_control::FrameStats;

TEST(FrameStats, empty_window)
{
  FrameStats stats;
  stats.reset(10.0);
  const auto s = stats.take(11.0);
  EXPECT_DOUBLE_EQ(s.period, 1.0);
  EXPECT_EQ(s.frames, 0u);
  EXPECT_EQ(s.dropped, 0u);
  EXPECT_DOUBLE_EQ(s.rate, 0.0);
  EXPECT_DOUBLE_EQ(s.latency_mean, 0.0);
}

TEST(FrameStats, rate_and_latency)
{
  FrameStats stats;
  stats.reset(0.0);
  for (uint64_t i = 1; i <= 100; i++) {
    stats.add_frame(i, i <= 50 ? 0.002 : 0.004);
  }
  const auto s = stats.take(0.5);
  EXPECT_EQ(s.frames, 100u);
  EXPECT_EQ(s.dropped, 0u);
  EXPECT_DOUBLE_EQ(s.rate, 200.0);
  EXPECT_NEAR(s.latency_mean, 0.003, 1e-12);
  EXPECT_DOUBLE_EQ(s.latency_max, 0.004);
}

TEST(FrameStats, gaps_count_as_drops)
{
  FrameStats stats;
  stats.reset(0.0);
  stats.add_frame(10, 0.0);   // first frame never counts drops
  stats.add_frame(11, 0.0);
  stats.add_frame(15, 0.0);   // 12, 13, 14 missing
  stats.add_frame(16, 0.0);
  const auto s = stats.take(1.0);
  EXPECT_EQ(s.frames, 4u);
  EXPECT_EQ(s.dropped, 3u);
}

TEST(FrameStats, duplicates_ignored_and_restart_resyncs)
{
  FrameStats stats;
  stats.reset(0.0);
  stats.add_frame(100, 0.0);
  stats.add_frame(100, 0.0);  // duplicate
  stats.add_frame(3, 0.0);    // mocap software restarted
  stats.add_frame(4, 0.0);
  const auto s = stats.take(1.0);
  EXPECT_EQ(s.frames, 3u);
  EXPECT_EQ(s.dropped, 0u);
}

TEST(FrameStats, windows_and_totals)
{
  FrameStats stats;
  stats.reset(0.0);
  stats.add_frame(1, 0.0);
  stats.add_frame(3, 0.0);
  auto s = stats.take(1.0);
  EXPECT_EQ(s.frames, 2u);
  EXPECT_EQ(s.dropped, 1u);

  // The frame sequence continues across windows.
  stats.add_frame(4, 0.0);
  stats.add_frame(6, 0.0);
  s = stats.take(3.0);
  EXPECT_DOUBLE_EQ(s.period, 2.0);
  EXPECT_EQ(s.frames, 2u);
  EXPECT_EQ(s.dropped, 1u);
  EXPECT_EQ(s.total_frames, 4u);
  EXPECT_EQ(s.total_dropped, 2u);

  stats.reset(5.0);
  s = stats.take(6.0);
  EXPECT_EQ(s.total_frames, 0u);
  stats.add_frame(1000, 0.0);  // no drops after reset
  EXPECT_EQ(stats.take(7.0).dropped, 0u);
}

TEST(FrameStats, tracking_ratio_per_body)
{
  FrameStats stats;
  stats.reset(0.0);
  for (int i = 0; i < 4; i++) {
    stats.add_tracking("robot", i != 0);
    stats.add_tracking("wand", false);
  }
  auto s = stats.take(1.0);
  ASSERT_EQ(s.tracking_ratio.size(), 2u);
  EXPECT_DOUBLE_EQ(s.tracking_ratio["robot"], 0.75);
  EXPECT_DOUBLE_EQ(s.tracking_ratio["wand"], 0.0);
  // Bodies not seen in the next window disappear.
  EXPECT_TRUE(stats.take(2.0).tracking_ratio.empty());
}

TEST(FrameStats, negative_period_is_clamped)
{
  FrameStats stats;
  stats.reset(10.0);
  stats.add_frame(1, 0.0);
  const auto s = stats.take(9.0);
  EXPECT_DOUBLE_EQ(s.period, 0.0);
  EXPECT_DOUBLE_EQ(s.rate, 0.0);
}
