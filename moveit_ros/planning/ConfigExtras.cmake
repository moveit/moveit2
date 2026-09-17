# Extras module needed for dependencies to find boost components

find_package(Boost CONFIG REQUIRED COMPONENTS filesystem date_time
                                              program_options thread chrono)
