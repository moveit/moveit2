add_library(simple_sampler_plugin SHARED
  src/simple_sampler.cpp
)
set_target_properties(simple_sampler_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(simple_sampler_plugin ${THIS_PACKAGE_INCLUDE_DEPENDS})
