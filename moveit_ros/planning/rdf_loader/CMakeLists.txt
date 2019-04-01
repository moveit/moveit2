set(MOVEIT_LIB_NAME moveit_rdf_loader)

add_library(${MOVEIT_LIB_NAME} src/rdf_loader.cpp)
set_target_properties(${MOVEIT_LIB_NAME} PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")


install(TARGETS ${MOVEIT_LIB_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
  
install(DIRECTORY include/ DESTINATION include)
