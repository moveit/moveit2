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
#include <stdexcept>

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
      },
      [](ExecutionJob::Generation, std::exception_ptr) {}));

  EXPECT_FALSE(execution.start([&](ExecutionJob::Generation) { second_job_ran = true; }, [] {},
                               [](ExecutionJob::Generation, std::exception_ptr) {}));
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
        },
        [](ExecutionJob::Generation, std::exception_ptr) {}));
  }

  EXPECT_TRUE(canceled);
  EXPECT_TRUE(worker_completed);
}

TEST(ExecutionJob, CancelAndWaitInvalidatesCompletion)
{
  ExecutionJob execution;
  ExecutionJob::Generation generation = 0;
  ASSERT_TRUE(execution.start([&](ExecutionJob::Generation value) { generation = value; }, [] {},
                              [](ExecutionJob::Generation, std::exception_ptr) {}));

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
  ASSERT_TRUE(execution.start([&](ExecutionJob::Generation value) { first_generation = value; }, [] {},
                              [](ExecutionJob::Generation, std::exception_ptr) {}));
  execution.wait();
  ASSERT_TRUE(execution.isCurrent(first_generation));

  ASSERT_TRUE(execution.start([&](ExecutionJob::Generation value) { second_generation = value; }, [] {},
                              [](ExecutionJob::Generation, std::exception_ptr) {}));
  execution.wait();

  EXPECT_FALSE(execution.isCurrent(first_generation));
  EXPECT_TRUE(execution.isCurrent(second_generation));
}

TEST(ExecutionJob, ReportsExceptionsAndAllowsRestart)
{
  ExecutionJob execution;
  ExecutionJob::Generation job_generation = 0;
  ExecutionJob::Generation exception_generation = 0;
  std::exception_ptr exception;

  ASSERT_TRUE(execution.start(
      [&](ExecutionJob::Generation generation) {
        job_generation = generation;
        throw std::runtime_error("execution failed");
      },
      [] {},
      [&](ExecutionJob::Generation generation, std::exception_ptr caught_exception) {
        exception_generation = generation;
        exception = caught_exception;
      }));
  execution.wait();

  EXPECT_FALSE(execution.active());
  EXPECT_EQ(job_generation, exception_generation);
  ASSERT_NE(exception, nullptr);
  EXPECT_THROW(std::rethrow_exception(exception), std::runtime_error);

  bool restarted_job_ran = false;
  ASSERT_TRUE(execution.start([&](ExecutionJob::Generation) { restarted_job_ran = true; }, [] {},
                              [](ExecutionJob::Generation, std::exception_ptr) {}));
  execution.wait();
  EXPECT_TRUE(restarted_job_ran);
}

TEST(ExecutionJob, ContainsExceptionHandlerFailures)
{
  ExecutionJob execution;
  ASSERT_TRUE(execution.start([](ExecutionJob::Generation) { throw std::runtime_error("execution failed"); }, [] {},
                              [](ExecutionJob::Generation, std::exception_ptr) {
                                throw std::runtime_error("reporting failed");
                              }));

  execution.wait();
  EXPECT_FALSE(execution.active());
}
}  // namespace moveit_rviz_plugin
