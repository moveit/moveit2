/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage nor the names of its
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

/* Author: Ioan Sucan */

#include <moveit/pick_place/manipulation_pipeline.h>

namespace pick_place
{
rclcpp::Logger LOGGER_MANIPULATION_PIPELINE =
    rclcpp::get_logger("moveit_ros_manipulation").get_child("manipulation_pipeline");
ManipulationPipeline::ManipulationPipeline(const std::string& name, unsigned int nthreads)
  : name_(name), nthreads_(nthreads), verbose_(false), stop_processing_(true)
{
  processing_threads_.resize(nthreads, nullptr);
}

ManipulationPipeline::~ManipulationPipeline()
{
  reset();
}

ManipulationPipeline& ManipulationPipeline::addStage(const ManipulationStagePtr& next)
{
  next->setVerbose(verbose_);
  stages_.push_back(next);
  return *this;
}

const ManipulationStagePtr& ManipulationPipeline::getFirstStage() const
{
  if (stages_.empty())
  {
    static const ManipulationStagePtr EMPTY;
    return EMPTY;
  }
  else
    return stages_.front();
}

const ManipulationStagePtr& ManipulationPipeline::getLastStage() const
{
  if (stages_.empty())
  {
    static const ManipulationStagePtr EMPTY;
    return EMPTY;
  }
  else
    return stages_.back();
}

void ManipulationPipeline::reset()
{
  clear();
  stages_.clear();
}

void ManipulationPipeline::setVerbose(bool flag)
{
  verbose_ = flag;
  for (std::size_t i = 0; i < stages_.size(); ++i)
    stages_[i]->setVerbose(flag);
}

void ManipulationPipeline::clear()
{
  stop();
  {
    boost::mutex::scoped_lock slock(queue_access_lock_);
    queue_.clear();
  }
  {
    boost::mutex::scoped_lock slock(result_lock_);
    success_.clear();
    failed_.clear();
  }
}

void ManipulationPipeline::start()
{
  stop_processing_ = false;
  empty_queue_threads_ = 0;
  for (std::size_t i = 0; i < stages_.size(); ++i)
    stages_[i]->resetStopSignal();
  for (std::size_t i = 0; i < processing_threads_.size(); ++i)
    if (!processing_threads_[i])
      processing_threads_[i] = new boost::thread(boost::bind(&ManipulationPipeline::processingThread, this, i));
}

void ManipulationPipeline::signalStop()
{
  for (std::size_t i = 0; i < stages_.size(); ++i)
    stages_[i]->signalStop();
  stop_processing_ = true;
  queue_access_cond_.notify_all();
}

void ManipulationPipeline::stop()
{
  signalStop();
  for (std::size_t i = 0; i < processing_threads_.size(); ++i)
    if (processing_threads_[i])
    {
      processing_threads_[i]->join();
      delete processing_threads_[i];
      processing_threads_[i] = nullptr;
    }
}

void ManipulationPipeline::processingThread(unsigned int index)
{
  RCLCPP_DEBUG(LOGGER_MANIPULATION_PIPELINE, "Start thread %d for '%s'", index, name_.c_str());

  while (!stop_processing_)
  {
    bool inc_queue = false;
    boost::unique_lock<boost::mutex> ulock(queue_access_lock_);
    // if the queue is empty, we trigger the corresponding event
    if (queue_.empty() && !stop_processing_ && empty_queue_callback_)
    {
      empty_queue_threads_++;
      inc_queue = true;
      if (empty_queue_threads_ == processing_threads_.size())
        empty_queue_callback_();
    }
    while (queue_.empty() && !stop_processing_)
      queue_access_cond_.wait(ulock);
    while (!stop_processing_ && !queue_.empty())
    {
      ManipulationPlanPtr g = queue_.front();
      queue_.pop_front();
      if (inc_queue)
      {
        empty_queue_threads_--;
        inc_queue = false;
      }

      queue_access_lock_.unlock();
      try
      {
        g->error_code_.val = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
        for (std::size_t i = 0; !stop_processing_ && i < stages_.size(); ++i)
        {
          bool res = stages_[i]->evaluate(g);
          g->processing_stage_ = i + 1;
          if (!res)
          {
            boost::mutex::scoped_lock slock(result_lock_);
            failed_.push_back(g);
            RCLCPP_INFO(LOGGER_MANIPULATION_PIPELINE, "Manipulation plan %d failed at stage '%s' on thread %d", g->id_,
                        stages_[i]->getName().c_str(), index);
            break;
          }
        }
        if (g->error_code_.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
        {
          g->processing_stage_++;
          {
            boost::mutex::scoped_lock slock(result_lock_);
            success_.push_back(g);
          }
          signalStop();
          RCLCPP_INFO(LOGGER_MANIPULATION_PIPELINE, "Found successful manipulation plan!");
          if (solution_callback_)
            solution_callback_();
        }
      }
      catch (std::exception& ex)
      {
        RCLCPP_ERROR(LOGGER_MANIPULATION_PIPELINE, "[%s:%u] %s", name_.c_str(), index, ex.what());
      }
      queue_access_lock_.lock();
    }
  }
}

void ManipulationPipeline::push(const ManipulationPlanPtr& plan)
{
  boost::mutex::scoped_lock slock(queue_access_lock_);
  queue_.push_back(plan);
  RCLCPP_INFO(LOGGER_MANIPULATION_PIPELINE, "Added plan for pipeline '%s'. Queue is now of size %d", name_.c_str(),
              queue_.size());
  queue_access_cond_.notify_all();
}

void ManipulationPipeline::reprocessLastFailure()
{
  boost::mutex::scoped_lock slock(queue_access_lock_);
  if (failed_.empty())
    return;
  ManipulationPlanPtr plan = failed_.back();
  failed_.pop_back();
  plan->clear();
  queue_.push_back(plan);
  RCLCPP_INFO(LOGGER_MANIPULATION_PIPELINE, "Re-added last failed plan for pipeline '%s'. Queue is now of size %f",
              name_.c_str(), queue_.size());
  queue_access_cond_.notify_all();
}
}  // namespace pick_place
