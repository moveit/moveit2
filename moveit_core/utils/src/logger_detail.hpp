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
/// Returns a mutex the caller must lock before reading or writing `node`
/// afterwards, so a concurrent caller and pre-shutdown callback can't race
/// on it.
///
/// Two independent, deliberate design choices:
///
/// A. Node capture: the callback references the caller-owned `node` slot
///    itself, rather than strongly capturing the Node. A strong Node
///    capture could create Context -> pre-shutdown callback -> Node ->
///    Context: Context strongly owns every pre-shutdown callback registered
///    on it, and Node (via its NodeBase) strongly owns its
///    rclcpp::Context::SharedPtr, so a callback holding a shared_ptr to the
///    Node would close that cycle. A plain reference cannot itself be part
///    of a shared_ptr reference cycle, so this can't happen.
///
/// B. Guard/mutex capture: the callback captures the returned mutex by
///    *weak_ptr*, not shared_ptr, so that in the "rclcpp::shutdown() is
///    never called" static-destruction path, the caller-owned mutex --
///    constructed immediately after `node`, so by the standard's
///    reverse-order-of-completed-construction rule it is destroyed *before*
///    `node` -- is already gone by the time `node` itself is destroyed. The
///    weak_ptr lock then fails if the callback fires while `node` is being
///    (or has been) destroyed, so the callback never touches the `node`
///    slot while its own static shared_ptr is itself being torn down,
///    leaving `node` to be destroyed exactly as it would have been before
///    this fix.
inline std::shared_ptr<std::mutex> registerNodeResetOnPreShutdown(rclcpp::Node::SharedPtr& node)
{
  auto mutex = std::make_shared<std::mutex>();
  std::weak_ptr<std::mutex> weak_mutex = mutex;
  node->get_node_base_interface()->get_context()->add_pre_shutdown_callback([weak_mutex, &node] {
    std::shared_ptr<std::mutex> locked = weak_mutex.lock();
    if (!locked)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(*locked);
    // Drop the reference so ~rclcpp::Node runs now, while the RMW context is
    // still alive, instead of racing against it at static destruction time.
    node.reset();
  });
  return mutex;
}

}  // namespace detail
}  // namespace moveit
