/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*      Title     : test_butterworth_filter.cpp
 *      Project   : moveit_core
 *      Created   : 07/21/2020
 *      Author    : Adam Pettinger
 *      Desc      : Unit test for moveit::ButterworthFilter
 */

#include <gtest/gtest.h>
#include <moveit/online_signal_smoothing/butterworth_filter.h>

TEST(SMOOTHING_PLUGINS, FilterConverge)
{
  online_signal_smoothing::ButterworthFilter lpf(2.0);
  EXPECT_DOUBLE_EQ(0.0, lpf.filter(0.0));
  double value;
  for (size_t i = 0; i < 100; ++i)
  {
    value = lpf.filter(5.0);
  }
  // Check that the filter converges to expected value after many identical messages
  EXPECT_DOUBLE_EQ(5.0, value);

  // Then check that a different measurement changes the value
  EXPECT_NE(5.0, lpf.filter(100.0));
}

TEST(SMOOTHING_PLUGINS, FilterReset)
{
  online_signal_smoothing::ButterworthFilter lpf(2.0);
  EXPECT_DOUBLE_EQ(0.0, lpf.filter(0.0));
  lpf.reset(5.0);
  double value = lpf.filter(5.0);

  // Check that the filter was properly set to the desired value
  EXPECT_DOUBLE_EQ(5.0, value);

  // Then check that a different measurement changes the value
  EXPECT_NE(5.0, lpf.filter(100.0));
}
