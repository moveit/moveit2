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

// Regression test for moveit/moveit2#3827: node-backed loggers must be
// destroyed before their rclcpp::Context calls rcl_shutdown(), not left to
// run their destructor at static-destruction time -- and must not do so via
// a reference cycle that would leak instead if rclcpp::shutdown() is never
// called.
//
// registerNodeResetOnPreShutdown() lives in the internal (non-installed)
// logger_detail.hpp, included by both logger.cpp (compiled into the real
// moveit_utils library) and this test, so the white-box tests below
// exercise the exact same code the library uses. This test links against
// the real moveit_utils rather than recompiling logger.cpp, and does not
// expose anything through the public moveit/utils/logger.hpp API.
//
// getGlobalRootLogger()'s before-rclcpp::init() fallback is covered
// separately, by its own single-test executable/process
// (test_logger_before_init.cpp) -- not here, since that behavior can only
// be observed by the first thing in a process to touch it.
#include "../src/logger_detail.hpp"

#include <moveit/utils/logger.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <future>
#include <memory>
#include <string>

namespace
{

// Each white-box test uses a fresh private context so neither callbacks
// nor initialized logger state can carry over from another test.
rclcpp::NodeOptions makeOptionsWithFreshContext(std::shared_ptr<rclcpp::Context>& context_out)
{
  context_out = std::make_shared<rclcpp::Context>();
  context_out->init(0, nullptr);
  rclcpp::NodeOptions options;
  options.context(context_out);
  return options;
}

TEST(RegisterNodeResetOnPreShutdownTest, ExplicitShutdownDestroysNode)
{
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::NodeOptions options = makeOptionsWithFreshContext(context);

  bool destroyed_while_context_valid = false;
  rclcpp::Node::SharedPtr node(new rclcpp::Node("logger_reset_test", options), [&](rclcpp::Node* ptr) {
    destroyed_while_context_valid = context->is_valid();
    delete ptr;
  });
  std::weak_ptr<rclcpp::Node> weak_node = node;
  auto registration = moveit::detail::registerNodeResetOnPreShutdown(node);

  EXPECT_FALSE(weak_node.expired());

  // rcl_shutdown() runs as part of this call. If the node were destroyed
  // only afterwards (e.g. at static destruction), an RMW implementation
  // whose own process-wide state is torn down around the same time (e.g.
  // rmw_zenoh_cpp) could abort. The pre-shutdown callback must destroy the
  // node first.
  context->shutdown("test shutdown");

  EXPECT_EQ(node, nullptr) << "the caller's own node slot must be reset by the callback";
  EXPECT_TRUE(weak_node.expired()) << "node must be destroyed before rcl_shutdown(), not after";
  EXPECT_TRUE(destroyed_while_context_valid) << "the node deleter must run before RMW shutdown";
}

// Regression test for a race CodeRabbit flagged in getGlobalRootLogger():
// once registerNodeResetOnPreShutdown() returns, a concurrent
// rclcpp::shutdown() can reset the node at any point afterwards, including
// before the caller's first read of it. This deterministically exercises
// the worst case of that race -- the reset having already happened by the
// time the read takes its lock -- without needing actual concurrent
// threads (which would make the test flaky). It proves the lock-then-check
// pattern getGlobalRootLogger() and setNodeLoggerName() both use is safe:
// once the same mutex the callback locks is held, the node is never
// dereferenced without first being checked for null.
TEST(RegisterNodeResetOnPreShutdownTest, LockedReadAfterResetDoesNotDereferenceNull)
{
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::NodeOptions options = makeOptionsWithFreshContext(context);

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("race_test_node", options);
  auto registration = moveit::detail::registerNodeResetOnPreShutdown(node);

  // Simulate a shutdown racing ahead of the first locked read.
  context->shutdown("simulate a shutdown racing ahead of the first read");
  ASSERT_EQ(node, nullptr);

  std::lock_guard<std::mutex> lock(*registration.getMutex());
  EXPECT_FALSE(static_cast<bool>(node)) << "node must be safely observed as reset while holding the lock";
}

// Registration destruction must remove the callback and release its guard
// without retaining the node when explicit shutdown is omitted.
TEST(RegisterNodeResetOnPreShutdownTest, DoesNotRetainNodeOrGuardWithoutExplicitShutdown)
{
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::NodeOptions options = makeOptionsWithFreshContext(context);

  std::weak_ptr<rclcpp::Node> weak_node;
  std::weak_ptr<std::mutex> weak_mutex;
  {
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("cycle_test_node", options);
    weak_node = node;
    auto registration = moveit::detail::registerNodeResetOnPreShutdown(node);
    weak_mutex = registration.getMutex();
    EXPECT_FALSE(weak_node.expired());
    EXPECT_FALSE(weak_mutex.expired());
    // Registration is destroyed before node, just as with the library's
    // function-local statics during unloading or process exit.
  }

  EXPECT_TRUE(weak_mutex.expired()) << "mutex must not be kept alive by the callback's capture of it";
  EXPECT_TRUE(weak_node.expired()) << "node must not be kept alive by the callback's capture of it";

  // The context can still shut down after its registration owner is gone.
  context->shutdown("test cleanup");
}

TEST(RegisterNodeResetOnPreShutdownTest, DestructionRemovesOnlyItsCallback)
{
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::NodeOptions options = makeOptionsWithFreshContext(context);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("registration_scope_test", options);
  bool other_callback_called = false;
  context->add_pre_shutdown_callback([&] { other_callback_called = true; });
  const auto callbacks_before = context->get_pre_shutdown_callbacks().size();

  {
    auto registration = moveit::detail::registerNodeResetOnPreShutdown(node);
    EXPECT_EQ(context->get_pre_shutdown_callbacks().size(), callbacks_before + 1);
  }

  EXPECT_EQ(context->get_pre_shutdown_callbacks().size(), callbacks_before);
  EXPECT_NE(node, nullptr) << "unregistering must leave the caller-owned node intact";
  node.reset();
  EXPECT_TRUE(context->shutdown("shutdown after registration removal"));
  EXPECT_TRUE(other_callback_called);
}

TEST(RegisterNodeResetOnPreShutdownTest, ReleasesContextAfterUnregistering)
{
  std::weak_ptr<rclcpp::Context> weak_context;
  rclcpp::Node::SharedPtr node;
  std::unique_ptr<moveit::detail::NodeResetOnPreShutdown> registration;
  {
    std::shared_ptr<rclcpp::Context> context;
    rclcpp::NodeOptions options = makeOptionsWithFreshContext(context);
    weak_context = context;
    node = std::make_shared<rclcpp::Node>("context_owner_test", options);
    registration = std::make_unique<moveit::detail::NodeResetOnPreShutdown>(node);
  }

  node.reset();
  EXPECT_FALSE(weak_context.expired()) << "registration keeps the context alive until removal";
  registration.reset();
  EXPECT_TRUE(weak_context.expired()) << "registration must not create a context ownership cycle";
}

// Run under test_logger's CTest process timeout as a final bound for a mutex
// deadlock. The deliberately blocked node deleter also has its own timeout.
// This tests registration removal with library code still mapped; it does
// not attempt concurrent dlclose while a callback is executing.
TEST(RegisterNodeResetOnPreShutdownTest, RemovalWaitsForActiveShutdownCallback)
{
  using namespace std::chrono_literals;
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::NodeOptions options = makeOptionsWithFreshContext(context);
  std::promise<void> node_deletion_started;
  auto node_deletion_started_future = node_deletion_started.get_future();
  std::promise<void> release_node_deletion;
  auto release_node_deletion_future = release_node_deletion.get_future();
  rclcpp::Node::SharedPtr node(new rclcpp::Node("concurrent_removal_test", options), [&](rclcpp::Node* ptr) {
    node_deletion_started.set_value();
    EXPECT_EQ(release_node_deletion_future.wait_for(5s), std::future_status::ready);
    delete ptr;
  });
  const auto callbacks_before = context->get_pre_shutdown_callbacks().size();
  auto registration = std::make_unique<moveit::detail::NodeResetOnPreShutdown>(node);

  auto shutdown = std::async(std::launch::async, [&] { return context->shutdown("concurrent removal"); });
  const auto deletion_started = node_deletion_started_future.wait_for(5s);
  EXPECT_EQ(deletion_started, std::future_status::ready);

  std::promise<void> removal_started;
  auto removal_started_future = removal_started.get_future();
  auto removal = std::async(std::launch::async, [&] {
    removal_started.set_value();
    registration.reset();
  });
  EXPECT_EQ(removal_started_future.wait_for(5s), std::future_status::ready);
  if (deletion_started == std::future_status::ready)
  {
    EXPECT_EQ(removal.wait_for(50ms), std::future_status::timeout)
        << "removal must wait while the callback is using the node slot and mutex";
  }
  release_node_deletion.set_value();

  EXPECT_EQ(shutdown.wait_for(5s), std::future_status::ready);
  EXPECT_EQ(removal.wait_for(5s), std::future_status::ready);
  EXPECT_TRUE(shutdown.get());
  removal.get();
  EXPECT_EQ(node, nullptr);
  EXPECT_EQ(context->get_pre_shutdown_callbacks().size(), callbacks_before);
}

// Black-box test of the actual public API, using the process-wide default
// rclcpp context (the same one moveit::setNodeLoggerName() and
// moveit::getGlobalRootLogger() use internally via plain rclcpp::init()),
// rather than a private test context. This exercises the same
// init/use/shutdown sequence as the issue #3827 reporter's MWE, plus a
// second call afterwards to guard against a post-shutdown null dereference.
TEST(SetNodeLoggerNameTest, SafeAcrossExplicitShutdown)
{
  rclcpp::init(0, nullptr);

  moveit::setNodeLoggerName("logger_black_box_test");
  const rclcpp::Logger logger_before_shutdown = moveit::getLogger("child");
  ASSERT_NE(logger_before_shutdown.get_name(), nullptr);
  const std::string name_before_shutdown = logger_before_shutdown.get_name();
  try
  {
    RCLCPP_INFO(logger_before_shutdown, "before shutdown");
  }
  catch (const std::exception& ex)
  {
    FAIL() << "logging before shutdown must not throw: " << ex.what();
  }

  rclcpp::shutdown();

  // The pre-shutdown callback has now reset setNodeLoggerName()'s node, but
  // the rclcpp::Logger previously assigned into getGlobalRootLogger() is
  // unaffected: rclcpp::Logger owns its logger-name state independently and
  // does not retain a reference to the Node, so its name -- and every
  // logger derived from it via get_child() -- is still valid and unchanged
  // here.
  const rclcpp::Logger logger_after_shutdown = moveit::getLogger("child");
  ASSERT_NE(logger_after_shutdown.get_name(), nullptr);
  EXPECT_STREQ(logger_after_shutdown.get_name(), name_before_shutdown.c_str());

  // A second call after shutdown, and continuing to log through the (now
  // node-less) logger, must not crash: this is a regression guard for the
  // "first call wins" static being reset out from under a later caller. The
  // name is deliberately unchanged from before shutdown: with the node
  // already reset, setNodeLoggerName() leaves getGlobalRootLogger() as-is
  // rather than dereferencing the destroyed node.
  try
  {
    moveit::setNodeLoggerName("logger_black_box_test_after_shutdown");
    const rclcpp::Logger logger_after_second_call = moveit::getLogger("child");
    ASSERT_NE(logger_after_second_call.get_name(), nullptr);
    EXPECT_STREQ(logger_after_second_call.get_name(), name_before_shutdown.c_str())
        << "setNodeLoggerName() must not change the logger after its node has already been reset";
    RCLCPP_INFO(logger_after_second_call, "after shutdown");
  }
  catch (const std::exception& ex)
  {
    FAIL() << "setNodeLoggerName()/logging after shutdown must not throw: " << ex.what();
  }
}

}  // namespace

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
