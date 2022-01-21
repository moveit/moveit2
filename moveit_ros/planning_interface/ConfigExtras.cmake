# Extras module needed for dependencies to find boost components

#if(Boost_VERSION LESS 106700)
#  set(BOOST_PYTHON_COMPONENT python)
#else()
#  set(BOOST_PYTHON_COMPONENT python${PYTHON_VERSION_MAJOR}${PYTHON_VERSION_MINOR})
#endif()

find_package(Boost REQUIRED COMPONENTS
  date_time
  filesystem
  program_options
  #${BOOST_PYTHON_COMPONENT}
  system
  thread
)
