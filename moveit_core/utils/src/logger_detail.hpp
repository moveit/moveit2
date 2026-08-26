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

// Internal implementation detail of logger.cpp, split out into its own
// non-installed header purely so test_logger.cpp can test it directly
// without exposing it through the public moveit/utils/logger.hpp API.
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <mutex>

namespace moveit
{
namespace detail
{

/// Out-of-band flag + mutex used to guard a caller-owned rclcpp::Node::SharedPtr
/// against a pre-shutdown callback resetting it concurrently, and to record
/// whether that reset has already happened. Deliberately its own, separately
/// allocated object -- NOT a wrapper that also stores the Node::SharedPtr
/// itself. See registerNodeResetOnPreShutdown() for why.
struct LoggerNodeFlag
{
  std::mutex mutex;
  bool retired = false;
};

/// Arranges for `node` -- a caller-owned rclcpp::Node::SharedPtr with static
/// storage duration -- to be reset from an rclcpp pre-shutdown callback
/// (which runs *before* rcl_shutdown() tears down the associated RMW
/// context), instead of being left to run its destructor during static
/// destruction at process exit. Some RMW implementations (e.g. rmw_zenoh_cpp)
/// abort the process if RMW calls are made after their own process-wide
/// static state has already been torn down, which otherwise races against
/// the unspecified destruction order of unrelated function-local static
/// objects. See moveit/moveit2#3827.
///
/// Returns a LoggerNodeFlag the caller must lock (its mutex) before reading
/// or writing `node` afterwards, and can check (`retired`) to know whether
/// the callback already reset it.
///
/// Two deliberate design choices here, both required together and each
/// empirically verified while developing this fix:
///
/// 1. The callback captures `node` by *reference*, not by storing a
///    shared_ptr to it (or to a wrapper struct that also owns it). Even a
///    same-process, non-heap struct that bundles an rclcpp::Node::SharedPtr
///    together with any other member was observed to break ordinary Node
///    teardown at process exit when rclcpp::shutdown() is never called
///    explicitly -- independent of, and in addition to, the reference-cycle
///    concern below. Keeping `node` a fully standalone
///    rclcpp::Node::SharedPtr, exactly as in the pre-fix code, avoids that.
///    A plain reference cannot itself ever be part of a shared_ptr reference
///    cycle.
///
/// 2. The callback captures the returned LoggerNodeFlag by *weak_ptr*, not
///    shared_ptr. rclcpp::Node (via its NodeBase) strongly owns an
///    rclcpp::Context::SharedPtr, and Context strongly owns every
///    pre-shutdown callback registered on it. A callback holding a
///    shared_ptr to state that (transitively) owns the node would close a
///    strong reference cycle back to the context
///    (Context -> callback -> state -> node -> Context) that reference
///    counting alone could never break: if rclcpp::shutdown() were never
///    called, nothing would ever be destroyed at all, rather than merely
///    being destroyed later. A weak_ptr capture means the only edge from
///    Context back to this flag is non-owning, so there is no cycle.
///
/// Together, this also makes the callback's behavior when
/// rclcpp::shutdown() is never called explicitly well-defined: `node` is a
/// static, so by the time it and the returned flag (constructed
/// immediately afterwards, by the caller) reach static destruction, the
/// flag -- constructed later -- is destroyed first, by the standard's
/// reverse-order-of-completed-construction rule. That drops the callback's
/// only strong-refcounted reference *before* `node` is destroyed, so by the
/// time the callback could possibly fire afterwards (e.g. from within the
/// context's own destructor), locking the weak_ptr fails and the callback
/// safely does nothing, leaving `node` to be destroyed exactly as it would
/// have been before this fix -- not fixed, but not worsened either.
inline std::shared_ptr<LoggerNodeFlag> registerNodeResetOnPreShutdown(rclcpp::Node::SharedPtr& node)
{
  auto flag = std::make_shared<LoggerNodeFlag>();
  std::weak_ptr<LoggerNodeFlag> weak_flag = flag;
  node->get_node_base_interface()->get_context()->add_pre_shutdown_callback([weak_flag, &node] {
    std::shared_ptr<LoggerNodeFlag> locked = weak_flag.lock();
    if (!locked)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(locked->mutex);
    // Drop the reference so ~rclcpp::Node runs now, while the RMW context is
    // still alive, instead of racing against it at static destruction time.
    node.reset();
    locked->retired = true;
  });
  return flag;
}

}  // namespace detail
}  // namespace moveit
