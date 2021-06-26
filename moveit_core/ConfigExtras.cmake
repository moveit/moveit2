# Extras module needed for dependencies to find boost components

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
