# Extras module needed for dependencies to find boost components
# https://answers.ros.org/question/331089/ament_export_dependenciesboost-not-working/?answer=332460#post-id-332460

# boost::iostreams on Windows depends on boost::zlib
if(WIN32)
  set(EXTRA_BOOST_COMPONENTS zlib)
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
  ${EXTRA_BOOST_COMPONENTS}
)
