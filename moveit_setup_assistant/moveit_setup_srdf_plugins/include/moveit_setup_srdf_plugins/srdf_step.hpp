/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Metro Robots
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
 *   * Neither the name of Metro Robots nor the names of its
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

/* Author: David V. Lu!! */
#pragma once

#include <moveit_setup_framework/setup_step.hpp>
#include <moveit_setup_framework/data/srdf_config.hpp>

namespace moveit_setup
{
namespace srdf_setup
{
/**
 * @brief Setup Step that contains the SRDFConfig
 */
class SRDFStep : public SetupStep
{
public:
  void onInit() override
  {
    srdf_config_ = config_data_->get<SRDFConfig>("srdf");
  }

  bool isReady() const override
  {
    return srdf_config_->isConfigured();
  }

  bool hasGroups() const
  {
    return !srdf_config_->getGroups().empty();
  }

protected:
  std::shared_ptr<SRDFConfig> srdf_config_;
};

/**
 * @brief This class provides a number of standard operations based on srdf's vector members
 *
 * Assuming T is a type that has a name_ field, this provides the following operations on the container
 * in which the name_ field is kept unique.
 *  * find
 *  * create
 *  * rename
 *  * remove
 *  * get
 */
template <typename T>
class SuperSRDFStep : public SRDFStep
{
public:
  /**
   * @brief Returns the reference to the vector in the SRDF
   */
  virtual std::vector<T>& getContainer() = 0;

  /**
   * @brief Returns the info field associated with this part of the SRDF
   */
  virtual InformationFields getInfoField() const = 0;

  /**
   * @brief Return a pointer to an item with the given name if it exists, otherwise null
   */
  T* find(const std::string& name)
  {
    // Note: method is not const because otherwise we cannot return a non-const pointer
    for (T& item : getContainer())
    {
      if (item.name_ == name)
      {
        return &item;
      }
    }
    return nullptr;
  }

  /**
   * @brief Create an item with the given name and return the pointer
   * @note: Does not check if an item with the given name exists
   */
  T* create(const std::string& name)
  {
    T new_item;
    new_item.name_ = name;
    getContainer().push_back(new_item);
    srdf_config_->updateRobotModel(getInfoField());
    return &getContainer().back();
  }

  /**
   * @brief Renames an item and returns a pointer to the item
   * @throws runtime_error If an item exists with the new name
   */
  T* rename(const std::string& old_name, const std::string& new_name)
  {
    T* item = find(old_name);
    T* existing_item = find(new_name);
    if (existing_item)
    {
      throw std::runtime_error("An item already exists with that name!");
    }
    item->name_ = new_name;
    srdf_config_->updateRobotModel(getInfoField());
    return item;
  }

  /**
   * @brief Delete an item with the given name from the list
   * @return true if item was found
   */
  bool remove(const std::string& name)
  {
    auto& container = getContainer();
    for (auto it = container.begin(); it != container.end(); ++it)
    {
      if (it->name_ == name)  // string match
      {
        container.erase(it);
        srdf_config_->updateRobotModel(getInfoField());
        return true;
      }
    }
    return false;
  }

  /**
   * @brief Get a pointer to an item with the given name, creating if necessary.
   *        If old_name is provided (and is different) will rename the given item.
   */
  T* get(const std::string& name, const std::string& old_name = "")
  {
    if (name == old_name)
    {
      return find(name);
    }
    else if (old_name.empty())
    {
      return create(name);
    }
    else
    {
      return rename(old_name, name);
    }
  }
};
}  // namespace srdf_setup
}  // namespace moveit_setup
