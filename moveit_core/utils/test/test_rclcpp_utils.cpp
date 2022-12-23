/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, PickNik Inc.
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

#include <gtest/gtest.h>
#include <string>
#include <moveit/utils/rclcpp_utils.h>

TEST(sanitizeGraphResourceNameTest, RemovesDoubleSlashes)
{
  // Test that "//" is replaced with a single "/"
  std::string input = "path//to/file";
  std::string expected_output = "path/to/file";
  EXPECT_EQ(rclcpp::names::sanitizeGraphResourceName(input), expected_output);

  // Test that multiple instances of "//" are removed
  input = "path////to/file";
  expected_output = "path/to/file";
  EXPECT_EQ(rclcpp::names::sanitizeGraphResourceName(input), expected_output);
}

TEST(sanitizeGraphResourceNameTest, RemovesTrailingSlash)
{
  // Test that a trailing '/' is removed
  std::string input = "path/to/file/";
  std::string expected_output = "path/to/file";
  EXPECT_EQ(rclcpp::names::sanitizeGraphResourceName(input), expected_output);

  // Test that multiple trailing '/' characters are removed
  input = "path/to/file////";
  expected_output = "path/to/file";
  EXPECT_EQ(rclcpp::names::sanitizeGraphResourceName(input), expected_output);
}

TEST(sanitizeGraphResourceNameTest, HandlesEmptyInput)
{
  // Test that an empty string is returned for an empty input string
  std::string input = "";
  std::string expected_output = "";
  EXPECT_EQ(rclcpp::names::sanitizeGraphResourceName(input), expected_output);
}

TEST(appendAndSanitizeGraphResourceNameTest, CorrectlyAppendsAndCleansNames)
{
  // Test that "left" and "right" are correctly appended and cleaned
  std::string left = "path/to";
  std::string right = "file/";
  std::string expected_output = "path/to/file";
  EXPECT_EQ(rclcpp::names::appendAndSanitizeGraphResourceName(left, right), expected_output);

  // Test that "left" and "right" are correctly cleaned even if they contain "//" or trailing '/' characters
  left = "path//to/";
  right = "file////";
  expected_output = "path/to/file";
  EXPECT_EQ(rclcpp::names::appendAndSanitizeGraphResourceName(left, right), expected_output);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
