/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2021, PickNik Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/*      Title     : topic_resampler.hpp
 *      Project   : moveit_servo
 *      Created   : 10/31/2021
 *      Author    : Tyler Weaver
 */

#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <rclcpp/rclcpp.hpp>

namespace moveit_servo
{
template <class MessageT>
class TopicResampler
{
  rclcpp::Node::SharedPtr node_;
  std::string topic_;
  rclcpp::Duration period_;
  std::function<void(std::shared_ptr<MessageT>)> callback_;
  typename rclcpp::Subscription<MessageT>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex mutex_;
  std::shared_ptr<MessageT> msg_;

public:
  /**
   * @brief      Constructs a new topic resampler
   *
   * @param[in]  node      The ROS node
   * @param[in]  topic     The topic to subscribe to
   * @param[in]  period    The period to call the callback
   * @param[in]  callback  The callback to call
   */
  TopicResampler(rclcpp::Node::SharedPtr node, std::string topic, rclcpp::Duration period,
                 std::function<void(std::shared_ptr<MessageT>)> callback)
    : node_{ node }, topic_{ topic }, period_{ period }, callback_{ callback }
  {
    // Update the internal message state
    auto update_msg = [&](std::shared_ptr<MessageT> msg) -> void {
      const std::lock_guard<std::mutex> lock(mutex_);
      msg_ = msg;
    };

    // Publish the message (call the output callback)
    // Note that we make a deep copy of the message so we can call the callback outside
    // of the scope of the lock_guard.  This ensures that while the callback is executing
    // we don't have to hold the mutex and it won't block the subscriber.
    auto publish_msg = [&]() -> void {
      auto get_msg = [&]() -> std::shared_ptr<MessageT> {
        const std::lock_guard<std::mutex> lock(mutex_);
        return std::make_shared<MessageT>(*msg_);
      };
      callback_(get_msg());
    };

    // This happens on the first callback, here we start the timer that publishes the message.
    // The reason we wait until our first callback to do this is so that the timer does not call the
    // callback with an uninitalized message.  This also changes the subscription to just call the
    // update_msg lambda instead of the first_callback lambda.
    auto first_callback = [&, update_msg, publish_msg](std::shared_ptr<MessageT> msg) -> void {
      // Update the internal state and publish the first message
      update_msg(msg);
      publish_msg();

      // Start the resampling by changing the subscriber to just update the message
      sub_ = node_->create_subscription<MessageT>(topic_, rclcpp::SystemDefaultsQoS(), update_msg);
      // and create a timer that publishes the message at the specified period
      timer_ = node_->create_wall_timer(period_.to_chrono<std::chrono::milliseconds>(), publish_msg);
    };

    // Create the subscription for the first message
    sub_ = node_->create_subscription<MessageT>(topic_, rclcpp::SystemDefaultsQoS(), first_callback);
  }
};
}  // namespace moveit_servo
