# Extras module needed for dependencies to find boost components

find_package(
  Boost
  REQUIRED
  system
  date_time
  program_options
  thread)
