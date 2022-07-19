//
// Created by antoine on 11/07/22.
//

#ifndef SRC_OMPL_OPTIMIZATION_OBJECTIVE_LOADER_H
#define SRC_OMPL_OPTIMIZATION_OBJECTIVE_LOADER_H

#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/SpaceInformation.h>

#include <string>
#include <iostream>

#define MOVEIT_OPTIMIZATION_OBJECTIVE_PLUGIN(my_namespace, OptimizationObjective)                                      \
  namespace my_namespace                                                                                               \
  {                                                                                                                    \
  class OptimizationObjective##Loader : public ompl_optimization_loader::OptimizationObjectiveLoader                   \
  {                                                                                                                    \
  public:                                                                                                              \
    ompl::base::OptimizationObjectivePtr getOptimizationObjective(ompl::base::SpaceInformationPtr si) override         \
    {                                                                                                                  \
      return std::make_shared<OptimizationObjective>(si);                                                              \
    }                                                                                                                  \
  };                                                                                                                   \
  PLUGINLIB_EXPORT_CLASS(my_namespace::OptimizationObjective##Loader,                                                  \
                         ompl_optimization_loader::OptimizationObjectiveLoader)                                        \
  }

namespace ompl_optimization_loader
{
class OptimizationObjectiveLoader
{
public:
  OptimizationObjectiveLoader()
  {
  }
  ~OptimizationObjectiveLoader() = default;

  virtual ompl::base::OptimizationObjectivePtr getOptimizationObjective(ompl::base::SpaceInformationPtr si) = 0;
};
}  // namespace ompl_optimization_loader

#endif  // SRC_OMPL_OPTIMIZATION_OBJECTIVE_LOADER_H
