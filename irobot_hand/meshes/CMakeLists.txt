set (files 
finger_base.stl
finger_distal.stl
finger_proximal.stl
palm.stl
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/models/${MODEL_NAME}/meshes/)
