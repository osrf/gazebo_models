set(MODEL_NAME apollo15_landing_site_1000x1000)
set (files 
  model.config 
  model.sdf
)

add_subdirectory(materials)

install(FILES  DESTINATION /share//gazebo_models/environments//)
