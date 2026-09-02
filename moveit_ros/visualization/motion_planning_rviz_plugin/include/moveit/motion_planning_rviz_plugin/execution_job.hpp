/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2026, MoveIt Contributors
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
 *   * Neither the name of the copyright holder nor the names of its
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

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <utility>

namespace moveit_rviz_plugin
{
/** Owns one cancelable worker and always joins it before destruction. */
class ExecutionJob
{
public:
  using Generation = std::uint64_t;
  using Job = std::function<void(Generation)>;
  using CancelJob = std::function<void()>;

  ExecutionJob() = default;
  ExecutionJob(const ExecutionJob&) = delete;
  ExecutionJob& operator=(const ExecutionJob&) = delete;

  ~ExecutionJob()
  {
    cancelAndWait();
  }

  /** Start a job unless another job is still active. */
  bool start(Job job, CancelJob cancel_job)
  {
    std::scoped_lock lock(mutex_);
    if (active_)
      return false;

    if (thread_.joinable())
      thread_.join();
    const Generation generation = generation_.fetch_add(1) + 1;
    active_ = true;
    cancel_job_ = std::move(cancel_job);
    try
    {
      thread_ = std::thread([this, job = std::move(job), generation]() mutable {
        job(generation);
        active_ = false;
      });
    }
    catch (...)
    {
      generation_.fetch_add(1);
      active_ = false;
      cancel_job_ = {};
      throw;
    }
    return true;
  }

  void cancel()
  {
    CancelJob cancel_job;
    {
      std::scoped_lock lock(mutex_);
      if (active_)
        cancel_job = cancel_job_;
    }
    if (cancel_job)
      cancel_job();
  }

  void wait()
  {
    std::scoped_lock lock(mutex_);
    if (thread_.joinable())
      thread_.join();
    active_ = false;
    cancel_job_ = {};
  }

  void cancelAndWait()
  {
    std::scoped_lock lock(mutex_);
    generation_.fetch_add(1);
    if (active_ && cancel_job_)
      cancel_job_();
    if (thread_.joinable())
      thread_.join();
    active_ = false;
    cancel_job_ = {};
  }

  bool active() const
  {
    return active_;
  }

  bool isCurrent(Generation generation) const
  {
    return generation_.load() == generation;
  }

private:
  std::mutex mutex_;
  std::thread thread_;
  std::atomic_bool active_{ false };
  std::atomic<Generation> generation_{ 0 };
  CancelJob cancel_job_;
};
}  // namespace moveit_rviz_plugin
