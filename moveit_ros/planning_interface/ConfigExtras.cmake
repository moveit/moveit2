# Extras module needed for dependencies to find boost components

find_package(Boost REQUIRED COMPONENTS
  date_time
  filesystem
  program_options
  system
  thread
)
