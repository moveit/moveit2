/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, PickNik Inc.
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

// Regression test for moveit/moveit2#3827's before-rclcpp::init() fallback
// path. getGlobalRootLogger()'s underlying rclcpp::Logger is a
// function-local static computed exactly once per process, so this test
// must be the first thing in its process to touch it -- it is deliberately
// its own single-test executable (its own OS process under ctest), rather
// than a test case inside test_logger.cpp, so nothing else can call
// rclcpp::init() first regardless of gtest execution order.
//
// Uses the public API (moveit::getLogger()) rather than the private
// getGlobalRootLogger(), since getLogger() calls getGlobalRootLogger()
// internally (see logger.cpp) and so exercises the same fallback without
// needing to forward-declare an implementation-detail symbol.
#include <moveit/utils/logger.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

namespace
{

TEST(LoggerBeforeInitTest, GetLoggerFallsBackWithoutNode)
{
  ASSERT_FALSE(rclcpp::ok()) << "this test must run alone, in a fresh process, before rclcpp::init()";

  // moveit::getLogger() must not throw or crash when no rclcpp context has
  // ever been initialized; it should fall back to a plain, non-node logger.
  EXPECT_NO_THROW({ moveit::getLogger("child"); });

  const rclcpp::Logger logger = moveit::getLogger("child");
  EXPECT_NE(logger.get_name(), nullptr);
}

}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
