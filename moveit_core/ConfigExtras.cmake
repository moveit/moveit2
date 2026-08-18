# Extras module needed for dependencies to find boost components

find_package(
  Boost CONFIG REQUIRED
  COMPONENTS chrono
             date_time
             filesystem
             iostreams
             program_options
             regex
             serialization
             thread)
