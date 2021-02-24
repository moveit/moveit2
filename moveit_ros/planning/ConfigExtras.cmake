# Extras module needed for dependencies to find boost components
# https://answers.ros.org/question/331089/ament_export_dependenciesboost-not-working/?answer=332460#post-id-332460

find_package(Boost REQUIRED system filesystem date_time program_options thread chrono)
