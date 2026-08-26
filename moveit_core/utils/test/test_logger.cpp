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
// GetGlobalRootLoggerTest must run before SetNodeLoggerNameTest: the latter
// is the only test that touches the process-wide default rclcpp context
// (via plain rclcpp::init()), and getGlobalRootLogger()'s underlying logger
// is a function-local static that is only ever computed once for the life
// of the process. GoogleTest runs tests within one binary in the order they
// are defined (no shuffling is enabled here), so that ordering holds by
// construction; GetGlobalRootLoggerTest also asserts its own precondition
// (no context initialized yet) so a violation fails loudly instead of
// silently exercising the wrong code path.
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
  // Precondition for this test to mean anything: no rclcpp context has been
  // initialized yet in this process. getGlobalRootLogger()'s underlying
  // logger is a function-local static computed once per process, so if this
  // ever runs after some other test has called rclcpp::init(), it would
  // silently stop testing the "before init" fallback path. Fail loudly
  // instead of passing for the wrong reason.
  ASSERT_FALSE(rclcpp::ok());

  // getGlobalRootLogger() must not throw or crash when no rclcpp context has
  // ever been initialized; it should fall back to a plain, non-node logger.
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
  // Must stay in scope (not just be constructed) until after context->shutdown()
  // below: it is what the pre-shutdown callback's weak_ptr needs to lock
  // successfully in order to reset `node`.
  std::shared_ptr<std::mutex> mutex = moveit::detail::registerNodeResetOnPreShutdown(node);

  EXPECT_FALSE(weak_node.expired());

  // rcl_shutdown() runs as part of this call. If the node were destroyed
  // only afterwards (e.g. at static destruction), an RMW implementation
  // whose own process-wide state is torn down around the same time (e.g.
  // rmw_zenoh_cpp) could abort. The pre-shutdown callback must destroy the
  // node first.
  context->shutdown("test shutdown");

  EXPECT_EQ(node, nullptr) << "the caller's own node slot must be reset by the callback";
  EXPECT_TRUE(weak_node.expired()) << "node must be destroyed before rcl_shutdown(), not after";
}

// Proves the callback does not strongly capture either the node or the
// mutex, by exercising the "rclcpp::shutdown() is never called" path and
// checking actual object destruction via weak_ptr expiry (not merely a
// process exit code):
//
// - if the callback strongly captured the mutex, weak_mutex would not
//   expire while the context (which owns the callback) remained alive;
// - if the callback strongly captured the Node, weak_node would not expire
//   either, since the context owns the callback and the Node owns the
//   context;
// - with the intended weak/non-owning captures, both expire once the
//   caller's own `node` and `mutex` variables go out of scope, even though
//   the (still-alive) context's registered callback references them.
TEST(RegisterNodeResetOnPreShutdownTest, DoesNotRetainNodeOrGuardWithoutExplicitShutdown)
{
  std::shared_ptr<rclcpp::Context> context;
  rclcpp::NodeOptions options = makeOptionsWithFreshContext(context);

  std::weak_ptr<rclcpp::Node> weak_node;
  std::weak_ptr<std::mutex> weak_mutex;
  {
    rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("cycle_test_node", options);
    weak_node = node;
    std::shared_ptr<std::mutex> mutex = moveit::detail::registerNodeResetOnPreShutdown(node);
    weak_mutex = mutex;
    EXPECT_FALSE(weak_node.expired());
    EXPECT_FALSE(weak_mutex.expired());
    // Both `node` and `mutex` (the only strong owners of the node and the
    // mutex, respectively) go out of scope here, *without* ever calling
    // context->shutdown() -- this is the "no explicit shutdown" case.
  }

  EXPECT_TRUE(weak_mutex.expired()) << "mutex must not be kept alive by the callback's capture of it";
  EXPECT_TRUE(weak_node.expired()) << "node must not be kept alive by the callback's capture of it";

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
