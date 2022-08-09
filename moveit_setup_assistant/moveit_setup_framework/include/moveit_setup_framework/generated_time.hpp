/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Metro Robots
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
 *   * Neither the name of Metro Robots nor the names of its
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

/* Author: David V. Lu!! */

#pragma once

#include <filesystem>
#include <string>

namespace moveit_setup
{
/**
 * This file contains a bunch of conversions for converting
 * std::filesystem::file_time_type (a.k.a. the return type from std::filesystem::last_write_time)
 * to other useful types and back
 */
using GeneratedTime = std::filesystem::file_time_type;

/**
 * @brief Convert a time_point from an arbitrary clock to GeneratedTime
 */
template <typename CLOCK>
inline GeneratedTime convertTime(const typename CLOCK::time_point& t0)
{
  return std::chrono::time_point_cast<GeneratedTime::duration>(t0 - CLOCK::now() + GeneratedTime::clock::now());
}

/**
 * @brief Convert GeneratedTime to a time_point from an arbitrary clock
 */
template <typename CLOCK>
inline typename CLOCK::time_point convertTime(const GeneratedTime& t0)
{
  return std::chrono::time_point_cast<typename CLOCK::duration>(t0 - GeneratedTime::clock::now() + CLOCK::now());
}

/**
 * @brief Convert an integral epoch from an arbitrary clock to GeneratedTime
 */
template <typename CLOCK>
inline GeneratedTime convertTime(long int epoch)
{
  auto t0 = CLOCK::from_time_t(epoch);
  return convertTime<CLOCK>(t0);
}

/**
 * @brief Convert an integral epoch to GeneratedTime (using system_clock)
 */
inline GeneratedTime fromEpoch(long int epoch)
{
  return convertTime<std::chrono::system_clock>(epoch);
}

/**
 * @brief Convert a GeneratedTime to a std::time_t (using system_clock)
 */
template <typename CLOCK>
inline std::time_t toStandardTime(const GeneratedTime& t)
{
  return CLOCK::to_time_t(convertTime<CLOCK>(t));
}

/**
 * @brief Convert a GeneratedTime to an integral epoch (using system_clock)
 */
inline long int toEpoch(const GeneratedTime& t)
{
  return toStandardTime<std::chrono::system_clock>(t);
}

/**
 * @brief Convert a GeneratedTime to a std::string (using system_clock)
 */
inline std::string toString(const GeneratedTime& t)
{
  std::time_t tt = toStandardTime<std::chrono::system_clock>(t);
  char buff[20];
  strftime(buff, 20, "%Y-%m-%d %H:%M:%S", localtime(&tt));
  return std::string(buff);
}

}  // namespace moveit_setup
