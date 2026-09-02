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

#include <moveit/motion_planning_rviz_plugin/execution_job.hpp>

#include <gtest/gtest.h>

#include <condition_variable>
#include <mutex>

namespace moveit_rviz_plugin
{
TEST(ExecutionJob, RejectsConcurrentJob)
{
  std::mutex mutex;
  std::condition_variable condition;
  bool release = false;
  bool second_job_ran = false;

  ExecutionJob execution;
  ASSERT_TRUE(execution.start(
      [&](ExecutionJob::Generation) {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, [&] { return release; });
      },
      [&] {
        {
          std::scoped_lock lock(mutex);
          release = true;
        }
        condition.notify_one();
      }));

  EXPECT_FALSE(execution.start([&](ExecutionJob::Generation) { second_job_ran = true; }, [] {}));
  execution.cancelAndWait();
  EXPECT_FALSE(second_job_ran);
  EXPECT_FALSE(execution.active());
}

TEST(ExecutionJob, DestructionCancelsBeforeJoining)
{
  std::mutex mutex;
  std::condition_variable condition;
  bool canceled = false;
  bool worker_completed = false;

  {
    ExecutionJob execution;
    ASSERT_TRUE(execution.start(
        [&](ExecutionJob::Generation) {
          std::unique_lock<std::mutex> lock(mutex);
          condition.wait(lock, [&] { return canceled; });
          worker_completed = true;
        },
        [&] {
          {
            std::scoped_lock lock(mutex);
            canceled = true;
          }
          condition.notify_one();
        }));
  }

  EXPECT_TRUE(canceled);
  EXPECT_TRUE(worker_completed);
}

TEST(ExecutionJob, CancelAndWaitInvalidatesCompletion)
{
  ExecutionJob execution;
  ExecutionJob::Generation generation = 0;
  ASSERT_TRUE(execution.start([&](ExecutionJob::Generation value) { generation = value; }, [] {}));

  execution.wait();
  ASSERT_TRUE(execution.isCurrent(generation));
  execution.cancelAndWait();
  EXPECT_FALSE(execution.isCurrent(generation));
}

TEST(ExecutionJob, NewJobInvalidatesPreviousCompletion)
{
  ExecutionJob execution;
  ExecutionJob::Generation first_generation = 0;
  ExecutionJob::Generation second_generation = 0;
  ASSERT_TRUE(execution.start([&](ExecutionJob::Generation value) { first_generation = value; }, [] {}));
  execution.wait();
  ASSERT_TRUE(execution.isCurrent(first_generation));

  ASSERT_TRUE(execution.start([&](ExecutionJob::Generation value) { second_generation = value; }, [] {}));
  execution.wait();

  EXPECT_FALSE(execution.isCurrent(first_generation));
  EXPECT_TRUE(execution.isCurrent(second_generation));
}
}  // namespace moveit_rviz_plugin
