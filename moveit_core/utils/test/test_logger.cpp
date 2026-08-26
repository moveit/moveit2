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
// registerNodeResetOnPreShutdown()/LoggerNodeFlag live in the internal
// (non-installed) logger_detail.hpp, included by both logger.cpp (compiled
// into the real moveit_utils library) and this test, so the white-box tests
// below exercise the exact same code the library uses. This test links
// against the real moveit_utils rather than recompiling logger.cpp, and does
// not expose anything through the public moveit/utils/logger.hpp API.
//
// Test order matters in this file: GetGlobalRootLoggerTest must run before
// SetNodeLoggerNameTest, since the latter is the only test that touches the
// process-wide default rclcpp context (via plain rclcpp::init()), and
// getGlobalRootLogger()'s underlying logger is a function-local static that
// is only ever computed once for the life of the process. GoogleTest runs
// tests within one binary in the order they are defined (no shuffling is
// enabled here), so that ordering is preserved by construction.
#include "../src/logger_detail.hpp"

#include <moveit/utils/logger.hpp>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <string>

namespace moveit
{
// getGlobalRootLogger() has external linkage (it is not static/anonymous),
// but is intentionally not declared in the public logger.hpp -- it is an
// implementation detail of setNodeLoggerName()/getLogger(). Forward-declare
// it here to exercise its no-init fallback path directly, without adding it
// to the public header.
rclcpp::Logger& getGlobalRootLogger();
}  // namespace moveit

namespace
{

TEST(GetGlobalRootLoggerTest, FallsBackToNonNodeLoggerBeforeInit)
{
  // getGlobalRootLogger() must not throw or crash when no rclcpp context has
  // ever been initialized; it should fall back to a plain, non-node logger.
  // (getGlobalRootLogger()'s underlying logger is only ever computed once
  // per process, so this only meaningfully verifies the "before init" path
  // if it runs before anything else in this binary calls rclcpp::init() --
  // see the file comment above.)
  EXPECT_NO_THROW({ rclcpp::Logger logger = moveit::getGlobalRootLogger(); });
}

// Each white-box test below uses its own private rclcpp::Context (rather
// than the process default one) so tests are fully isolated from one
// another: rclcpp pre/on-shutdown callbacks are never removed from a
// Context once registered and persist across repeated init() calls on that
// same context, so reusing the global default context across tests would
// let one test's callback fire again during a later test's shutdown.
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

  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("logger_reset_test", options);
  std::weak_ptr<rclcpp::Node> weak_node = node;
  std::shared_ptr<moveit::detail::LoggerNodeFlag> flag = moveit::detail::registerNodeResetOnPreShutdown(node);

  EXPECT_FALSE(weak_node.expired());
  EXPECT_FALSE(flag->retired);

  // rcl_shutdown() runs as part of this call. If the node were destroyed
  // only afterwards (e.g. at static destruction), an RMW implementation
  // whose own process-wide state is torn down around the same time (e.g.
  // rmw_zenoh_cpp) could abort. The pre-shutdown callback must destroy the
  // node first.
  context->shutdown("test shutdown");

  EXPECT_EQ(node, nullptr) << "the caller's own node slot must be reset by the callback";
  EXPECT_TRUE(weak_node.expired()) << "node must be destroyed before rcl_shutdown(), not after";
  EXPECT_TRUE(flag->retired);
}

// Proves there is no Context -> callback -> flag -> node -> Context
// reference cycle: if there were, dropping the caller's (only remaining
// strong) reference to the node without ever calling context->shutdown()
// would not be enough to free it, because the context's still-registered
// callback would still be keeping it alive. This tests actual object
// destruction via weak_ptr expiry, not merely a process exit code.
TEST(RegisterNodeResetOnPreShutdownTest, NoReferenceCycleWithoutExplicitShutdown)
{
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::NodeOptions options = makeOptionsWithFreshContext(context);

  std::weak_ptr<rclcpp::Node> weak_node;
  std::weak_ptr<moveit::detail::LoggerNodeFlag> weak_flag;
  {
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("cycle_test_node", options);
    weak_node = node;
    std::shared_ptr<moveit::detail::LoggerNodeFlag> flag = moveit::detail::registerNodeResetOnPreShutdown(node);
    weak_flag = flag;
    EXPECT_FALSE(weak_node.expired());
    EXPECT_FALSE(weak_flag.expired());
    // Both `node` and `flag` (the only strong owners of the node and the
    // flag, respectively) go out of scope here, *without* ever calling
    // context->shutdown() -- this is the "no explicit shutdown" case.
  }

  // If the pre-shutdown callback held a strong reference back to the flag
  // (or the flag held one to the node) that the context kept alive, these
  // would still report `false`: the context is still alive and would still
  // be keeping them alive through that callback.
  EXPECT_TRUE(weak_flag.expired()) << "flag must not be kept alive by a reference cycle through the context";
  EXPECT_TRUE(weak_node.expired()) << "node must not be kept alive by a reference cycle through the context";

  // The context itself, and its now-dangling (weak-only, already-expired)
  // callback registration, can be safely torn down too.
  context->shutdown("test cleanup");
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
  try
  {
    RCLCPP_INFO(moveit::getLogger("child"), "before shutdown");
  }
  catch (const std::exception& ex)
  {
    FAIL() << "logging before shutdown must not throw: " << ex.what();
  }

  rclcpp::shutdown();

  // A second call after shutdown, and continuing to log through the (now
  // node-less) logger, must not crash: this is a regression guard for the
  // "first call wins" static being reset out from under a later caller.
  try
  {
    moveit::setNodeLoggerName("logger_black_box_test_after_shutdown");
    RCLCPP_INFO(moveit::getLogger("child"), "after shutdown");
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
