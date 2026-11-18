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
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
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

// Linux-only process test. The executable has no moveit_utils dependency:
// dlopen loads a bridge that links the actual built library. Each CTest case
// runs in a fresh process because both logger entry points use static state.
#include <dlfcn.h>
#include <link.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

namespace
{
std::string canonicalPath(const char* path)
{
  std::unique_ptr<char, decltype(&std::free)> resolved(realpath(path, nullptr), &std::free);
  return resolved ? resolved.get() : "";
}

bool mappedLibrary(const std::string& expected)
{
  struct Search
  {
    const std::string& expected;
    bool found = false;
  } search{ expected };
  dl_iterate_phdr(
      [](dl_phdr_info* info, size_t, void* data) {
        auto& search = *static_cast<Search*>(data);
        if (info->dlpi_name && canonicalPath(info->dlpi_name) == search.expected)
        {
          search.found = true;
          return 1;
        }
        return 0;
      },
      &search);
  return search.found;
}

TEST(LoggerUnloadTest, LoggerCallbackLifetime)
{
  const char* entry = std::getenv("MOVEIT_LOGGER_UNLOAD_ENTRY");
  const char* order = std::getenv("MOVEIT_LOGGER_UNLOAD_ORDER");
  ASSERT_NE(entry, nullptr);
  ASSERT_NE(order, nullptr);
  const bool set_name = std::strcmp(entry, "set") == 0;
  ASSERT_TRUE(set_name || std::strcmp(entry, "get") == 0);
  const bool unload_first = std::strcmp(order, "unload-first") == 0;
  const bool shutdown_first = std::strcmp(order, "shutdown-first") == 0;
  ASSERT_TRUE(unload_first || shutdown_first || std::strcmp(order, "no-unload") == 0);

  const std::string library_path = canonicalPath(MOVEIT_UTILS_PATH);
  ASSERT_FALSE(library_path.empty());
  ASSERT_FALSE(mappedLibrary(library_path)) << "test executable must not retain a direct or extra library reference";
  rclcpp::init(0, nullptr);
  auto context = rclcpp::contexts::get_global_default_context();
  const auto callbacks_before = context->get_pre_shutdown_callbacks().size();
  void* bridge = dlopen(LOGGER_UNLOAD_DUT_PATH, RTLD_NOW | RTLD_LOCAL);
  ASSERT_NE(bridge, nullptr) << dlerror();
  using UseLogger = void (*)(bool);
  using LoggerSymbol = std::uintptr_t (*)();
  auto use_logger = reinterpret_cast<UseLogger>(dlsym(bridge, "moveit_logger_dut_use"));
  auto logger_symbol = reinterpret_cast<LoggerSymbol>(dlsym(bridge, "moveit_logger_dut_symbol"));
  ASSERT_NE(use_logger, nullptr);
  ASSERT_NE(logger_symbol, nullptr);
  Dl_info info{};
  ASSERT_NE(dladdr(reinterpret_cast<void*>(logger_symbol()), &info), 0);
  ASSERT_EQ(canonicalPath(info.dli_fname), library_path) << "wrong MoveIt library loaded";
  ASSERT_TRUE(mappedLibrary(library_path));

  use_logger(set_name);
  ASSERT_EQ(context->get_pre_shutdown_callbacks().size(), callbacks_before + (set_name ? 2u : 1u));
  if (shutdown_first)
  {
    ASSERT_TRUE(rclcpp::shutdown());
  }
  if (unload_first || shutdown_first)
  {
    ASSERT_EQ(dlclose(bridge), 0);
    bridge = nullptr;
    if (mappedLibrary(library_path))
    {
      GTEST_SKIP() << "dlclose returned but the shared library remained mapped; this loader cannot exercise unloading";
    }
  }
  if (!shutdown_first)
  {
    ASSERT_TRUE(rclcpp::shutdown());
  }
  if (unload_first || shutdown_first)
  {
    EXPECT_EQ(context->get_pre_shutdown_callbacks().size(), callbacks_before);
  }
  // The no-unload control intentionally retains its loader reference until
  // process exit, including destruction of the default ROS context.
}
}  // namespace
