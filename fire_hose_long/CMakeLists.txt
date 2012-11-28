set(MODEL_NAME fire_hose)
add_subdirectory(meshes)
set (files 
  model.sdf
  model_short.sdf
  model_1link.sdf
  manifest.xml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/models/${MODEL_NAME}/)
