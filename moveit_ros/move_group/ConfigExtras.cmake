# Extras module needed for dependencies to find boost components

find_package(
  Boost
  REQUIRED
  random
  system
  filesystem
  date_time
  program_options
  thread)
