  // gazebo_controllers.yaml ------------------------------------------------------------------
  file.file_name_ = "gazebo_controllers.yaml";
  file.rel_path_ = config_data_->appendPaths(config_path, file.file_name_);
  template_path = config_data_->appendPaths(config_data_->template_package_path_, file.rel_path_);
  file.description_ = "Configuration of Gazebo controllers";
  file.gen_func_ = std::bind(&ConfigurationFilesWidget::copyTemplate, this, template_path, std::placeholders::_1);

  // demo_gazebo.launch ------------------------------------------------------------------
  file.file_name_ = "demo_gazebo.launch";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, file.file_name_);
  file.description_ = "Run a demo of MoveIt with Gazebo and Rviz";
  file.gen_func_ = std::bind(&ConfigurationFilesWidget::copyTemplate, this, template_path, std::placeholders::_1);

  // gazebo.launch ------------------------------------------------------------------
  file.file_name_ = "gazebo.launch";
  file.rel_path_ = config_data_->appendPaths(launch_path, file.file_name_);
  template_path = config_data_->appendPaths(template_launch_path, "gazebo.launch");
  file.description_ = "Gazebo launch file which also launches ros_controllers and sends robot urdf to param server, "
                      "then using gazebo_ros pkg the robot is spawned to Gazebo";
  file.gen_func_ = std::bind(&ConfigurationFilesWidget::copyTemplate, this, template_path, std::placeholders::_1);
  file.write_on_changes = MoveItConfigData::SIMULATION;

  if (config_data_->save_gazebo_urdf_)
  {
    file.file_name_ = "gazebo_" + config_data_->urdf_model_->getName() + ".urdf";
    file.rel_path_ = config_data_->appendPaths(CONFIG_PATH, file.file_name_);
    file.description_ =
        "This <a href='https://wiki.ros.org/urdf'>URDF</a> file comprises your original robot description "
        "augmented with tags required for use with Gazebo, i.e. defining inertia and transmission properties. "
        "Checkout the <a href='http://gazebosim.org/tutorials/?tut=ros_urdf'>URDF Gazebo documentation</a> "
        "for more infos.";
    file.gen_func_ = std::bind(&MoveItConfigData::outputGazeboURDFFile, config_data_, std::placeholders::_1);
    file.write_on_changes = MoveItConfigData::SIMULATION;
    gen_files_.push_back(file);
  }




if (config_data_->save_gazebo_urdf_)
  {
    std::string file_name = "gazebo_" + config_data_->urdf_model_->getName() + ".urdf";
    std::string rel_path = config_data_->appendPaths(CONFIG_PATH, file_name);
    addTemplateString("[GAZEBO_URDF_LOAD_ATTRIBUTE]", "textfile=\"$(find " + new_package_name_ + ")/" + rel_path + "\"");
  }
  else  // reuse [URDF_LOAD_ATTRIBUTE] template
    addTemplateString("[GAZEBO_URDF_LOAD_ATTRIBUTE]", template_strings_.back().second);
