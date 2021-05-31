# Extras module needed for dependencies to find boost components

if(WIN32)
  # Fix linking errors on windows
  add_definitions(-DBOOST_ALL_NO_LIB)
  add_definitions(-DBOOST_ALL_DYN_LINK)
endif()
find_package(Boost REQUIRED
  chrono
  date_time
  filesystem
  iostreams
  program_options
  regex
  serialization
  system
  thread
)
